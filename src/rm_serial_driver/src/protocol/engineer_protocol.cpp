#include "rm_serial_driver/protocol/engineer_protocol.hpp"

namespace fyt::serial_driver::protocol{

    EngineerProtocol::EngineerProtocol(std::string_view port_name, bool enable_data_print) {
        auto uart_transporter = std::make_shared<UartTransporter>(std::string(port_name));
        if(uart_transporter->open() == false) {
            RCLCPP_ERROR(rclcpp::get_logger("EngineerProtocol"), "Failed to open port: %s", port_name.data());
        } else {
            RCLCPP_INFO(rclcpp::get_logger("EngineerProtocol"), "Successfully opened port: %s", port_name.data());
        }
        packet_tool_ = std::make_shared<FixedPacketTool<64>>(uart_transporter);
        packet_tool_->enableDataPrint(enable_data_print);
        packet_tool_->enableRealtimeSend(true);
    }

    // EngineerProtocol::~EngineerProtocol() = default;

    void EngineerProtocol::send(const moveit_msgs::msg::DisplayTrajectory &data, int sample_index) {
        RCLCPP_INFO(rclcpp::get_logger("EngineerProtocol"), "Sending trajectory point %d", sample_index);
        FixedPacket<64> packet;
        // packet自带头尾帧
        packet.loadData<uint8_t>(3, 1); // mode
        packet.loadData<float>(static_cast<float>(data.trajectory[0].joint_trajectory.points[sample_index].positions[0] + 0.3) * 1000, 2); // joint 1
        packet.loadData<float>(static_cast<float>(data.trajectory[0].joint_trajectory.points[sample_index].positions[1] - 3.141592), 6); // joint 2
        packet.loadData<float>(static_cast<float>(data.trajectory[0].joint_trajectory.points[sample_index].positions[2] - 3.141592), 10); // joint 3
        packet.loadData<float>(static_cast<float>(data.trajectory[0].joint_trajectory.points[sample_index].positions[3]), 14); // joint 4
        packet.loadData<float>(static_cast<float>(data.trajectory[0].joint_trajectory.points[sample_index].positions[4]), 18); // joint 5
        packet.loadData<float>(static_cast<float>(data.trajectory[0].joint_trajectory.points[sample_index].positions[5]), 22); // joint 6
        // CRC-16 校验由 FixedPacketTool::sendPacket() 自动完成
        packet_tool_->sendPacket(packet);
    }

    bool EngineerProtocol::receive(float data[16]) {
        FixedPacket<64> packet;
        bool recv_result = packet_tool_->recvPacket(packet);
        // 调试：每隔一段时间打印接收状态（避免刷屏）
        static int recv_count = 0;
        recv_count++;
        if (recv_count % 100 == 0) {  // 每100次打印一次
            RCLCPP_INFO(rclcpp::get_logger("EngineerProtocol"), "recvPacket called %d times, last result: %s", 
                recv_count, recv_result ? "SUCCESS" : "NO_DATA");
        }
        if (recv_result) {
            float raw_pos[6], raw_vel[6];
            uint8_t mode;
            packet.unloadData<uint8_t>(mode, 1);      // mode
            packet.unloadData<float>(raw_pos[0], 2);   // position joint 1 (mm)
            packet.unloadData<float>(raw_pos[1], 6);   // position joint 2
            packet.unloadData<float>(raw_pos[2], 10);  // position joint 3
            packet.unloadData<float>(raw_pos[3], 14);  // position joint 4
            packet.unloadData<float>(raw_pos[4], 18);  // position joint 5
            packet.unloadData<float>(raw_pos[5], 22);  // position joint 6
            packet.unloadData<float>(raw_vel[0], 26);  // velocity joint 1
            packet.unloadData<float>(raw_vel[1], 30);  // velocity joint 2
            packet.unloadData<float>(raw_vel[2], 34);  // velocity joint 3
            packet.unloadData<float>(raw_vel[3], 38);  // velocity joint 4
            packet.unloadData<float>(raw_vel[4], 42);  // velocity joint 5
            packet.unloadData<float>(raw_vel[5], 46);  // velocity joint 6

            data[0] = static_cast<float>(mode);
            data[1] = -(raw_pos[0] / 1000.0f) - 0.3f;
            data[2] = raw_pos[1] + 3.141592f;
            data[3] = raw_pos[2] + 3.141592f;
            data[4] = raw_pos[3];
            data[5] = raw_pos[4];
            data[6] = raw_pos[5];
            data[7] = raw_vel[0] / 1000.0f;
            data[8] = raw_vel[1];
            data[9] = raw_vel[2];
            data[10] = raw_vel[3];
            data[11] = raw_vel[4];
            data[12] = raw_vel[5];

            RCLCPP_INFO(rclcpp::get_logger("===========EngineerProtocol============"), "Received data: mode=%d, pos=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                static_cast<int>(data[0]), data[1], data[2], data[3], data[4], data[5], data[6]);

            return true;
        } else {
            return false;
        }
    }

    std::vector<rclcpp::SubscriptionBase::SharedPtr> EngineerProtocol::getSubscriptions(
    rclcpp::Node::SharedPtr node) {
    return {node->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
            "/display_planned_path",
            rclcpp::QoS(10),
            [this](const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg) { 
                RCLCPP_INFO(rclcpp::get_logger("EngineerProtocol"), "Received DisplayTrajectory message.");
                (void)msg;
                for (size_t sample_index = 0; sample_index < msg->trajectory[0].joint_trajectory.points.size(); sample_index++) {
                    this->send(*msg, sample_index);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                RCLCPP_INFO(rclcpp::get_logger("EngineerProtocol"), "Sent all trajectory points.");
            })

        };
    }
}