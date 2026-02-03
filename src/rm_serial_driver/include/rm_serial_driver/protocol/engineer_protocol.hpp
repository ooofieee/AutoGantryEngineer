#ifndef RM_SERIAL_DRIVER_PROTOCOL_ENGINEER_PROTOCOL_HPP_
#define RM_SERIAL_DRIVER_PROTOCOL_ENGINEER_PROTOCOL_HPP_

#include "rm_serial_driver/protocol.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"

namespace fyt::serial_driver::protocol {
// 默认
class EngineerProtocol : public Protocol {
public:
  explicit EngineerProtocol(std::string_view port_name, bool enable_data_print);

  ~EngineerProtocol() = default;
  
  // 实现基类的纯虚函数（保留接口兼容性）
  void send(const rm_interfaces::msg::GimbalCmd &data) override { (void)data; }
  bool receive(rm_interfaces::msg::SerialReceiveData &data) override { (void)data; return false; }
  
  // EngineerProtocol 特有的函数
  void send(const moveit_msgs::msg::DisplayTrajectory &data, int sample_index);
  bool receive(float data[16]);

  std::vector<rclcpp::SubscriptionBase::SharedPtr> getSubscriptions(rclcpp::Node::SharedPtr node) override;
  
  std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> getClients(
    rclcpp::Node::SharedPtr node) const override { (void)node; return {}; }

  std::string getErrorMessage() override { return packet_tool_->getErrorMessage(); }

private:
  FixedPacketTool<64>::SharedPtr packet_tool_;
};
}  // namespace fyt::serial_driver::protocol

#endif  // RM_SERIAL_DRIVER_PROTOCOL_ENGINEER_PROTOCOL_HPP_