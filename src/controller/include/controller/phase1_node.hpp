#ifndef PHASE1_NODE_HPP
#define PHASE1_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "controller/mtc.hpp"

class Phase1Node : public rclcpp::Node, public MTC {
public:
    Phase1Node(const std::string & name, const rclcpp::NodeOptions & options)
    : rclcpp::Node(name, options), MTC(options, name + "_mtc") {
        doTask();
    }

private:

    void setup_planning_scene() override{}

    mtc::Task create_task() override{
        mtc::Task task = TaskInit("push");

        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(0.1);
        cartesian_planner->setMaxAccelerationScalingFactor(0.1);
        cartesian_planner->setStepSize(0.01);

        {
            auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
            task.add(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject("cylinder_target", "link7");
            task.add(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("linear push", cartesian_planner);
            stage->setGroup("gantry_robot");
            stage->setIKFrame("link7");
            
            geometry_msgs::msg::Vector3Stamped direction;
            direction.header.frame_id = "cylinder_target_frame";
            direction.vector.x = 0.0;
            direction.vector.y = 0.1;
            direction.vector.z = 0.0;
            stage->setDirection(direction);
            
            task.add(std::move(stage));
        }
        return task;
    }
};

#endif // PHASE1_NODE_HPP