#include "rclcpp/rclcpp.hpp"
#include "controller/phase0_node.hpp"
#include "controller/phase1_node.hpp"
#include "controller/phase_test.hpp"

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto options = rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);

    auto phase0_node = std::make_shared<Phase0Node>("phase0_node", options);
    //auto phase1_node = std::make_shared<Phase1Node>("phase1_node", options);
    //auto phase_test_node = std::make_shared<PhaseTestNode>("phase_test_node", options);

    executor.add_node(phase0_node);
    //executor.add_node(phase1_node);
    //executor.add_node(phase_test_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}