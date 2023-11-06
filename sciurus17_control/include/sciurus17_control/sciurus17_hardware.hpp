// Copyright 2023 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef SCIURUS17_CONTROL__SCIURUS17_HARDWARE_HPP_
#define SCIURUS17_CONTROL__SCIURUS17_HARDWARE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "sciurus17_control/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rt_manipulators_cpp/hardware.hpp"
#include "rclcpp_lifecycle/state.hpp"

using hardware_interface::return_type;
using hardware_interface::CallbackReturn;

namespace sciurus17_control
{
class Sciurus17Hardware : public
  hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Sciurus17Hardware)

  SCIURUS17_CONTROL_PUBLIC
  ~Sciurus17Hardware();

  SCIURUS17_CONTROL_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  SCIURUS17_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  SCIURUS17_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  SCIURUS17_CONTROL_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  SCIURUS17_CONTROL_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  SCIURUS17_CONTROL_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  SCIURUS17_CONTROL_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool communication_timeout();

  std::shared_ptr<rt_manipulators_cpp::Hardware> hardware_;
  double timeout_seconds_;

  std::map<std::string, double> hw_position_commands_;
  std::map<std::string, double> hw_position_states_;
  std::map<std::string, double> hw_velocity_states_;
  std::map<std::string, double> hw_effort_states_;

  std::map<std::string, double> current_to_effort_;

  rclcpp::Clock steady_clock_;
  rclcpp::Time prev_comm_timestamp_;
  bool timeout_has_printed_;
};
}  // namespace sciurus17_control

#endif  // SCIURUS17_CONTROL__SCIURUS17_HARDWARE_HPP_