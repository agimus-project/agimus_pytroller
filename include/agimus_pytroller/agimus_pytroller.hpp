#ifndef AGIMUS_PYTROLLER_LOCAL__AGIMUS_PYTROLLER_LOCAL_HPP_
#define AGIMUS_PYTROLLER_LOCAL__AGIMUS_PYTROLLER_LOCAL_HPP_

#include <memory>
#include <string>
#include <vector>

#include <pybind11/embed.h>
#include <pybind11/numpy.h>

#include "agimus_pytroller/visibility.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <agimus_pytroller/agimus_pytroller_parameters.hpp>

namespace py = pybind11;

namespace agimus_pytroller {
class AgimusPytroller : public controller_interface::ControllerInterface {
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
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

protected:
  py::module_ python_module_;
  py::object controller_object_;
  py::object on_update_python_funct_;
  py::scoped_interpreter guard_;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::unique_ptr<py::array_t<double>> py_states_;

  std::vector<
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      ordered_command_interfaces_;

      std::vector<
      std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      ordered_state_interfaces_;
};

} // namespace agimus_pytroller

#endif // AGIMUS_PYTROLLER_LOCAL__AGIMUS_PYTROLLER_LOCAL_HPP_
