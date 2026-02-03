#pragma once
#include "orchestrator/phase_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/int8.hpp"

class Orchestrator
{
private:
    PhaseInterface phase;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr phase_publisher;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr state_accumulation_publisher;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr state_accumulation_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr phase_subscriber;
    rclcpp::Node::SharedPtr node_;
    int score;

public:
    Orchestrator(rclcpp::Node::SharedPtr node) : node_(node), score(0)
    {
        phase_publisher = node_->create_publisher<std_msgs::msg::Int8>("orchestrator/phase", 10);
        state_accumulation_publisher = node_->create_publisher<std_msgs::msg::Int8>("orchestrator/states", 10);
        state_accumulation_subscriber = node_->create_subscription<std_msgs::msg::Int8>(
            "orchestrator/states",
            10,
            [this](const std_msgs::msg::Int8::SharedPtr msg){
                if (msg->data == 1){
                    score++;
                    if (score % 2 == 0 && score <= 8){
                        phase.advancePhase();
                    }
                    else if (score > 8){
                        RCLCPP_INFO(node_->get_logger(), "All phases completed.");
                        score = 1;
                        phase.resetPhase();
                    }
                    auto phase_msg = std::make_shared<std_msgs::msg::Int8>();
                    phase_msg->data = static_cast<int8_t>(phase.getCurrentPhase());
                    phase_publisher->publish(*phase_msg);

                }
            });
        phase_subscriber = node_->create_subscription<std_msgs::msg::Int8>(
            "orchestrator/phase",
            10,
            [this](const std_msgs::msg::Int8::SharedPtr msg){
                // phase updater
            });
    }
    ~Orchestrator() = default;
};