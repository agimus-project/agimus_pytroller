#include <dlfcn.h>

// TODO fix imports order

#include "agimus_pytroller/agimus_pytroller.hpp"

#include <memory>
#include <ratio>
#include <string>
#include <vector>

#include <pybind11/embed.h>
#include <pybind11/numpy.h>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "std_msgs/msg/string.hpp"

namespace py = pybind11;

namespace agimus_pytroller {
AgimusPytroller::AgimusPytroller()
    : controller_interface::ControllerInterface(), guard_() {
  // TODO check if still needed
  dlopen("libpython3.10.so", RTLD_NOW | RTLD_GLOBAL);
}

controller_interface::CallbackReturn AgimusPytroller::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception &e) {
    fprintf(stderr, "Exception thrown when reading ROS parameters: %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    python_module_ = py::module_::import(params_.python_module.c_str());
  } catch (const std::exception &e) {
    fprintf(stderr, "Exception thrown when importing python module '%s': %s \n",
            params_.python_module.c_str(), e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AgimusPytroller::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  std_msgs::msg::String robot_description_msg;
  auto robot_description_sub =
      get_node()->create_subscription<std_msgs::msg::String>(
          "/robot_description", rclcpp::QoS(1).transient_local(),
          [](const std::shared_ptr<const std_msgs::msg::String>) {});

  const std::size_t retires = 10;
  for (std::size_t i = 0; i < retires; i++) {
    if (rclcpp::wait_for_message(robot_description_msg, robot_description_sub,
                                 get_node()->get_node_options().context(),
                                 std::chrono::seconds(1))) {
      RCLCPP_INFO(get_node()->get_logger(), "Robot description received.");
      break;
    } else if (i == retires) {
      fprintf(stderr, "Filed to receive data on '/robot_description' topic.");
      return controller_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_node()->get_logger(),
                "Robot description still not received. Retrying...");
  }

  try {
    controller_object_ =
        python_module_.attr("ControllerImpl")(robot_description_msg.data);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Filed to initialize python object 'ControllerImpl': %s\n ",
                 e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    on_update_python_funct_ = controller_object_.attr("on_update");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Filed to find 'ControllerImpl.on_update': %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
AgimusPytroller::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = params_.command_interfaces;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
AgimusPytroller::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::ALL};
}

controller_interface::CallbackReturn AgimusPytroller::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  py_states_ =
      std::make_unique<py::array_t<double>>(params_.state_interfaces.size());

  last_commands_.resize(params_.command_interfaces.size(), 0.0);

  first_python_call_ = true;

  if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, params_.command_interfaces, std::string(""),
          ordered_command_interfaces_)) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Expected %zu command interfaces, found %zu",
                 params_.command_interfaces.size(),
                 ordered_command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, params_.state_interfaces, std::string(""),
          ordered_state_interfaces_)) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Expected %zu state interfaces, found %zu",
                 params_.state_interfaces.size(),
                 ordered_state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  cancellation_token_ = false;
  control_spinner_thread_ =
      std::make_unique<std::thread>(&AgimusPytroller::py_control_spinner, this);
  RCLCPP_INFO(get_node()->get_logger(),
              "Controller thread spawned successfully");

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AgimusPytroller::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  cancellation_token_ = true;
  control_spinner_thread_->join();
  // TODO cleanup memory on deactivation
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
AgimusPytroller::update(const rclcpp::Time &time,
                        const rclcpp::Duration &period) {

  cycle_++;
  // Read last results of the controller
  if (cycle_ >= params_.python_downsample_factor || first_python_call_) {
    if (!first_python_call_) {
      // Convert hertz to nanoseconds
      const unsigned int offset = static_cast<unsigned int>(
          static_cast<double>(std::nano::den / this->get_update_rate()) * 0.75);

      const auto timeout = std::chrono::system_clock::time_point(
          std::chrono::nanoseconds(time.nanoseconds() + offset));

      bool timeout_reached = false;
      bool local_python_had_exception;
      {
        std::unique_lock lk(solver_stop_mtx_);
        // If false is returned this means solver did not finish on time
        timeout_reached = !solver_stop_cv_.wait_until(
            lk, timeout, [this] { return solver_finished_; });
        local_python_had_exception = python_had_exception_;
      }

      cycle_ = 0;
      if (timeout_reached) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Timeout reached before controller returned solution!");
        return controller_interface::return_type::ERROR;
      }
      if (local_python_had_exception) {
        return controller_interface::return_type::ERROR;
      }
    }
    first_python_call_ = false;

    for (std::size_t i = 0; i < ordered_command_interfaces_.size(); i++) {
      ordered_command_interfaces_[i].get().set_value(last_commands_[i]);
    }

    {
      std::lock_guard lk(solver_stop_mtx_);
      solver_finished_ = false;
    }

    {
      std::lock_guard lk(solver_start_mtx_);
      start_solver_ = true;
    }
    solver_start_cv_.notify_one();

  }
  return controller_interface::return_type::OK;
}

void AgimusPytroller::py_control_spinner() {
  {
    while (!cancellation_token_) {
      std::unique_lock lk(solver_start_mtx_);
      solver_start_cv_.wait(
          lk, [this] { return start_solver_ || cancellation_token_; });
      // If controller was stopped, stop the loop
      if (cancellation_token_) {
        break;
      }
      start_solver_ = false;

      // TODO find cleaner way to handle rewriting of the data
      auto state_buffer = py_states_->request();
      double *state_ptr = static_cast<double *>(state_buffer.ptr);
      for (std::size_t i = 0; i < ordered_state_interfaces_.size(); i++) {
        state_ptr[i] = ordered_state_interfaces_[i].get().get_value();
      }

      try {
        // TODO ensure there is no way to preallocate commands np.array
        const py::array_t<double> py_commands =
            on_update_python_funct_(*py_states_);
        const auto command_buffer = py_commands.request();
        const double *command_ptr = static_cast<double *>(command_buffer.ptr);

        {
          std::lock_guard lk(solver_stop_mtx_);
          python_had_exception_ = false;
          solver_finished_ = true;
          for (std::size_t i = 0; i < last_commands_.size(); i++) {
            last_commands_[i] = command_ptr[i];
          }
        }

      } catch (py::error_already_set &e) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Error when calling 'ControllerImpl.on_update()': %s",
                     e.what());
        {
          std::lock_guard lk(solver_stop_mtx_);
          python_had_exception_ = true;
          solver_finished_ = true;
          std::fill(last_commands_.begin(), last_commands_.end(),
                    std::numeric_limits<double>::signaling_NaN());
        }
      }
      solver_stop_cv_.notify_one();
    }
  }
}

} // namespace agimus_pytroller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(agimus_pytroller::AgimusPytroller,
                       controller_interface::ControllerInterface)
