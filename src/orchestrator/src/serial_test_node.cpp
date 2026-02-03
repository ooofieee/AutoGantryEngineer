#include "rm_serial_driver/protocol_factory.hpp"

class serial_test_node : public rclcpp::Node {
public:
    serial_test_node() : Node("serial_test_node") {
        protocol_ = fyt::serial_driver::ProtocolFactory::createProtocol("engineer", "/dev/ttyACM0", true);
        RCLCPP_INFO(this->get_logger(), "Serial test node initialized.");
    }

    void init() {
        subscriptions_ = protocol_->getSubscriptions(this->shared_from_this());
    }

private:
    std::unique_ptr<fyt::serial_driver::protocol::Protocol> protocol_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<serial_test_node>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}