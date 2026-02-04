// Copyright (c) 2022-2026, b»robotized group (template)
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

#ifndef GANTRY_GANTRY_ROBOT_HARDWARE_INTERFACE__GANTRY_ROBOT_HARDWARE_INTERFACE_HPP_
#define GANTRY_GANTRY_ROBOT_HARDWARE_INTERFACE__GANTRY_ROBOT_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

#include "rm_serial_driver/protocol/engineer_protocol.hpp"

namespace gantry_robot_hardware_interface
{

// 关节数量常量
constexpr size_t NUM_JOINTS = 6;

class GantryRobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GantryRobotHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void update_current_state();

private:
  // 关节位置命令和状态
  std::vector<double> hw_commands_position_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  
  std::unique_ptr<fyt::serial_driver::protocol::EngineerProtocol> protocol_;
  
  // 串口参数
  std::string serial_port_;
  bool enable_data_print_;
  
  // 通信状态
  std::atomic<bool> communication_active_{false};
  
  // 模式标志：mode == 3 时为 ros_control 模式，允许 write
  std::atomic<bool> is_ros_control_mode_{false};
  
  // JTC 控制器状态
  std::atomic<bool> jtc_active_{false};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
  
  // JTC 控制方法
  void deactivate_jtc();
  void activate_jtc();
  
  // 日志
  rclcpp::Logger logger_{rclcpp::get_logger("GantryRobotHardwareInterface")};
};

}  // namespace gantry_robot_hardware_interface

#endif  // GANTRY_GANTRY_ROBOT_HARDWARE_INTERFACE__GANTRY_ROBOT_HARDWARE_INTERFACE_HPP_
