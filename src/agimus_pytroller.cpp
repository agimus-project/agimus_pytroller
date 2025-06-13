#include <dlfcn.h>

#include <memory>
#include <ratio>
#include <string>
#include <vector>

#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/pytypes.h>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "std_msgs/msg/string.hpp"

#include "agimus_pytroller/agimus_pytroller.hpp"

namespace py = pybind11;

namespace agimus_pytroller {
AgimusPytroller::AgimusPytroller()
    : controller_interface::ControllerInterface(), guard_() {
  // Workaround to fix undeclared symbols for NumPy
  // https://stackoverflow.com/questions/49784583/numpy-import-fails-on-multiarray-extension-library-when-called-from-embedded-pyt
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

  try {
    on_message_python_funct_ = controller_object_.attr("on_message");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Filed to find 'ControllerImpl.on_message': %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  for (const std::string subscriber_name : params_.subscribed_topics) {
    const auto &topic_params =
        params_.subscribed_topics_map.at(subscriber_name);
    const auto &topic_type = topic_params.topic_type;
    const auto topic_name = topic_params.topic_name;

    topic_subscribers_.push_back(get_node()->create_generic_subscription(
        topic_name, topic_type, rclcpp::SystemDefaultsQoS(),
        [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          try {
            const auto serialized_msg = msg->get_rcl_serialized_message();
            std::vector<char> payload(serialized_msg.buffer,
                                      serialized_msg.buffer +
                                          serialized_msg.buffer_length);
            std::pair<std::string, std::vector<char>> message_data = {
                topic_name, payload};
            topic_queue_.push(message_data);
          } catch (const std::exception &e) {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Error calling message callback for topic: %s. Reason: %s\n ",
                topic_name.c_str(), e.what());
          }
        }));

    // Build the message map for given topic to avoid dynamic allocations
    try {
      controller_object_.attr("build_message_map")(
          topic_name, topic_type, topic_params.python_function_name);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Filed to find 'ControllerImpl.build_message_map': %s\n",
                   e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Without "priming" the controller, first message received takes enough time
  // to stall it for several calls
  try {
    controller_object_.attr("build_message_map")(
        "dummy_topic", "std_msgs/msg/String", "_dummy_topic_cb");
    auto message = std_msgs::msg::String();
    message.data = "dummy_text";

    rclcpp::Serialization<std_msgs::msg::String> msg_serializer;
    rclcpp::SerializedMessage serialized_msg;
    msg_serializer.serialize_message(&message, &serialized_msg);
    py::bytes msg_py_bytes(
        reinterpret_cast<const char *>(
            serialized_msg.get_rcl_serialized_message().buffer),
        serialized_msg.size());

    on_message_python_funct_("dummy_topic", msg_py_bytes);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Filed to perform priming call on 'ControllerImpl.on_message()': %s\n",
        e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    on_publish_python_funct_ = controller_object_.attr("on_publish");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Filed to find 'ControllerImpl.on_publish': %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  for (const std::string publisher_name : params_.published_topics) {
    const auto &topic_params = params_.published_topics_map.at(publisher_name);
    const auto &topic_type = topic_params.topic_type;
    const auto topic_name = topic_params.topic_name;

    topic_publishers_.push_back(std::make_pair(
        topic_name, get_node()->create_generic_publisher(
                        topic_name, topic_type, rclcpp::SystemDefaultsQoS())));

    // Build the message map for given topic to avoid dynamic allocations
    try {
      controller_object_.attr("build_message_map")(
          topic_name, topic_type, topic_params.python_msg_getter_name);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Filed to find 'ControllerImpl.build_message_map': %s\n",
                   e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    // Without "priming" the controller, first message received takes enough
    // time to stall it for several calls
    try {
      py::bytes data = on_publish_python_funct_(topic_name);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "Filed perform priming call on 'ControllerImpl.on_publish()': %s\n",
          e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
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

  last_state_.resize(params_.state_interfaces.size(), 0.0);
  new_commands_.resize(params_.command_interfaces.size(), 0.0);
  last_commands_.resize(params_.command_interfaces.size(), 0.0);
  new_commands_rt_.resize(params_.command_interfaces.size(), 0.0);

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
  raw_[2] = ordered_state_interfaces_[13].get().get_value();
  filer_[1] = (b_[0] * raw_[2] +
                b_[1] * raw_[1] +
                b_[2] * raw_[0] -
                a_[0] * filer_[1] -
                a_[1] * filer_[0]);
  raw_[0]  = raw_[1];
  raw_[1]  = raw_[2];
  filer_[0] = filer_[1];

  // Read last results of the controller
  if (cycle_ >= params_.python_downsample_factor || first_python_call_) {
    if (!first_python_call_) {
      bool timeout_reached = false;
      bool local_python_had_exception;

      {
        std::unique_lock lk(solver_stop_mtx_);
        // Do not block the thread to wait for spinner to finish
        if (params_.error_on_no_data) {
          if (!solver_finished_) {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Python controller did not manage to find solution on time!");
            return controller_interface::return_type::ERROR;
          }
        } else {
          solver_stop_cv_.wait(lk, [this] { return solver_finished_; });
        }
        // Reset the flag
        solver_finished_ = false;
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

    std::copy(new_commands_rt_.begin(), new_commands_rt_.end(),
              last_commands_.begin());
    std::copy(new_commands_.begin(), new_commands_.end(),
              new_commands_rt_.begin());

    for (std::size_t i = 0; i < ordered_state_interfaces_.size(); i++) {
      last_state_[i] = ordered_state_interfaces_[i].get().get_value();
    }
    last_state_[13] = filer_[1];

    start_solver_ = true;
  }

  if (params_.interpolate_trajectory) {
    // Linear interpolation factor between both trajectories.
    const double alpha = static_cast<double>(cycle_) /
                         static_cast<double>(params_.python_downsample_factor);
    for (std::size_t i = 0; i < ordered_command_interfaces_.size(); i++) {
      const double interp =
          (1.0 - alpha) * last_commands_[i] + alpha * new_commands_rt_[i];
      ordered_command_interfaces_[i].get().set_value(interp);
    }
  } else {
    for (std::size_t i = 0; i < ordered_command_interfaces_.size(); i++) {
      ordered_command_interfaces_[i].get().set_value(new_commands_rt_[i]);
    }
  }

  return controller_interface::return_type::OK;
}

void AgimusPytroller::py_control_spinner() {
  {
    while (!cancellation_token_) {
      while (true) {
        if (!topic_queue_.empty()) {
          const auto data = topic_queue_.front();
          py::bytes py_payload(data.second.data(), data.second.size());
          on_message_python_funct_(data.first, py_payload);
          topic_queue_.pop();
        }
        if (start_solver_ || cancellation_token_) {
          break;
        }
      }
      start_solver_ = false;

      // If controller was stopped, stop the loop
      if (cancellation_token_) {
        break;
      }
      auto state_buffer = py_states_->request();
      double *state_ptr = static_cast<double *>(state_buffer.ptr);
      std::copy(last_state_.begin(), last_state_.end(), state_ptr);

      try {
        {
          const py::array_t<double> py_commands =
              on_update_python_funct_(*py_states_);

          const auto command_buffer = py_commands.request();
          const double *command_ptr = static_cast<double *>(command_buffer.ptr);

          {
            std::lock_guard lk(solver_stop_mtx_);
            python_had_exception_ = false;
            solver_finished_ = true;
            for (std::size_t i = 0; i < new_commands_.size(); i++) {
              new_commands_[i] = command_ptr[i];
            }
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
          std::fill(new_commands_.begin(), new_commands_.end(),
                    std::numeric_limits<double>::signaling_NaN());
        }
      }
      solver_stop_cv_.notify_one();

      for (const auto &[topic_name, publisher] : topic_publishers_) {
        py::bytes data = on_publish_python_funct_(topic_name);

        char *serialized_buffer;
        Py_ssize_t length;
        if (PYBIND11_BYTES_AS_STRING_AND_SIZE(data.ptr(), &serialized_buffer,
                                              &length)) {
          RCLCPP_ERROR(get_node()->get_logger(),
                       "Filed to obtain serialized message from python for "
                       "topic '%s'!\n",
                       topic_name.c_str());
          continue;
        }
        if (length < 0) {
          RCLCPP_ERROR(get_node()->get_logger(),
                       "Filed to obtain serialized message from python for "
                       "topic '%s'!\n",
                       topic_name.c_str());
          continue;
        }

        rclcpp::SerializedMessage serialized_msg(length);
        std::memcpy(serialized_msg.get_rcl_serialized_message().buffer,
                    serialized_buffer, length);
        serialized_msg.get_rcl_serialized_message().buffer_length = length;
        serialized_msg.get_rcl_serialized_message().buffer_capacity = length;

        publisher->publish(serialized_msg);
      }
    }
  }
}

} // namespace agimus_pytroller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(agimus_pytroller::AgimusPytroller,
                       controller_interface::ControllerInterface)
