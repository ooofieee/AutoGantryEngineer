#pragma once
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit/task_constructor/stages.h"
#include "moveit/task_constructor/task.h"
#include "moveit/task_constructor/solvers.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit_task_constructor_msgs/action/execute_task_solution.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace mtc = moveit::task_constructor;

class MTC {
public:
    MTC (const rclcpp::NodeOptions & options) 
    : node_(std::make_shared<rclcpp::Node>("mtc_node", options)) {}

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() {
        return node_->get_node_base_interface();
    }

    ~MTC() = default;

    void doTask(){
        task_ = create_task();
        task_.enableIntrospection(true);

        try{
            task_.init();
        }
        catch (mtc::InitStageException& e){
            RCLCPP_ERROR_STREAM(LOGGER, e);
            return;
        }

        if (!task_.plan(5)){
            RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
            return;
        }
        task_.introspection().publishSolution(*task_.solutions().front());

        // Wait for the execute_task_solution action server provided by move_group + MTC capability
        if (!wait_for_execute_server(std::chrono::seconds(45))){
            RCLCPP_ERROR(LOGGER, "Failed to find execute_task_solution action server");
            return;
        }

        auto result = task_.execute(*task_.solutions().front());
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
            RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
            return;
        }

        return;
    }

    rclcpp::Node::SharedPtr getNodeSharedPtr(){
        return node_;
    }

    mtc::Task TaskInit(const std::string task_name){
        mtc::Task task;
        task.stages()->setName(task_name);
        task.loadRobotModel(MTC::getNodeSharedPtr());

        const auto& arm_group = "gantry_robot";
        const auto& hand_group = "gantry_hand";
        const auto& hand_frame = "link7";

        task.setProperty("group", arm_group);        
        task.setProperty("hand_group", hand_group);
        task.setProperty("eef", hand_frame);
        
        task.setProperty("execute_action_name", std::string("/execute_task_solution"));
        
        return task;
    }

    virtual void setup_planning_scene() = 0;
    virtual mtc::Task create_task() = 0;

private:
    bool wait_for_execute_server(std::chrono::seconds timeout){
        using ExecuteAction = moveit_task_constructor_msgs::action::ExecuteTaskSolution;
        std::array<std::string, 3> names = {"/execute_task_solution", "/move_group/execute_task_solution", "execute_task_solution"};

        const auto start = std::chrono::steady_clock::now();
        while (rclcpp::ok()){
            for (const auto& name : names){
                auto client = rclcpp_action::create_client<ExecuteAction>(node_, name);
                if (client->wait_for_action_server(std::chrono::seconds(1))){
                    RCLCPP_INFO(LOGGER, "Found action server: %s", name.c_str());
                    return true;
                }
            }

            const auto elapsed = std::chrono::steady_clock::now() - start;
            if (elapsed >= timeout){
                break;
            }

            RCLCPP_INFO(LOGGER, "Waiting for execute_task_solution action server...");
        }
        return false;
    }

    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger LOGGER = this->node_->get_logger();
};