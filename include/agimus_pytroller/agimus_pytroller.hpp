#ifndef AGIMUS_PYTROLLER_LOCAL__AGIMUS_PYTROLLER_LOCAL_HPP_
#define AGIMUS_PYTROLLER_LOCAL__AGIMUS_PYTROLLER_LOCAL_HPP_

#include <memory>
#include <queue>
#include <string>
#include <variant>
#include <vector>

#include <pybind11/embed.h>
#include <pybind11/numpy.h>

#include "agimus_pytroller/visibility.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <agimus_pytroller/agimus_pytroller_parameters.hpp>

namespace py = pybind11;

namespace agimus_pytroller {
class AgimusPytroller
    : public controller_interface::ChainableControllerInterface {
public:
  AGIMUS_PYTROLLER_LOCAL
  AgimusPytroller();

  AGIMUS_PYTROLLER_LOCAL
  ~AgimusPytroller() = default;

  AGIMUS_PYTROLLER_LOCAL
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  AGIMUS_PYTROLLER_LOCAL
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  AGIMUS_PYTROLLER_LOCAL
  controller_interface::CallbackReturn on_init() override;

  AGIMUS_PYTROLLER_LOCAL
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  AGIMUS_PYTROLLER_LOCAL
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  AGIMUS_PYTROLLER_LOCAL
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  AGIMUS_PYTROLLER_LOCAL
  bool on_set_chained_mode(bool chained_mode) override;

  AGIMUS_PYTROLLER_LOCAL
  controller_interface::return_type
  update_and_write_commands(const rclcpp::Time &time,
                            const rclcpp::Duration &period) override;

protected:
  using LoanedCommandInterface = hardware_interface::LoanedCommandInterface;
  using LoanedCommandInterfaceRef =
      std::reference_wrapper<LoanedCommandInterface>;

  using LoanedStateInterface = hardware_interface::LoanedStateInterface;
  using LoanedStateInterfaceRef = std::reference_wrapper<LoanedStateInterface>;

  std::vector<hardware_interface::CommandInterface>
  on_export_reference_interfaces() override;

  controller_interface::return_type
  update_reference_from_subscribers() override;

  void py_control_spinner();
  std::unique_ptr<std::thread> control_spinner_thread_;
  std::mutex solver_stop_mtx_;
  std::condition_variable solver_stop_cv_;
  std::atomic_bool cancellation_token_ = false;

  unsigned int update_rate_;

  unsigned int cycle_ = 0;
  bool first_python_call_ = true;
  std::atomic_bool start_solver_ = false;
  bool solver_finished_ = false;
  std::vector<double> last_state_;
  std::vector<double> last_commands_;
  std::vector<double> new_commands_;
  std::vector<double> new_commands_rt_;
  bool python_had_exception_ = false;

  py::module_ python_module_;
  py::object controller_object_;
  py::object on_update_python_funct_;
  py::object on_message_python_funct_;
  py::object on_publish_python_funct_;
  py::scoped_interpreter guard_;

  std::queue<std::pair<std::string, std::vector<char>>> topic_queue_;

  std::vector<std::shared_ptr<rclcpp::GenericSubscription>> topic_subscribers_;
  std::vector<std::pair<std::string, std::shared_ptr<rclcpp::GenericPublisher>>>
      topic_publishers_;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::unique_ptr<py::array_t<double>> py_states_;

  std::vector<LoanedCommandInterfaceRef> ordered_command_interfaces_;

  std::vector<hardware_interface::CommandInterface>
      command_reference_interfaces_;

  std::vector<hardware_interface::LoanedCommandInterface>
      loaned_reference_interfaces_;


  std::vector<std::variant<LoanedStateInterfaceRef, LoanedCommandInterfaceRef>>
      ordered_augmented_state_interfaces_;
};

} // namespace agimus_pytroller

#endif // AGIMUS_PYTROLLER_LOCAL__AGIMUS_PYTROLLER_LOCAL_HPP_
