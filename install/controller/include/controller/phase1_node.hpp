#ifndef PHASE1_NODE_HPP
#define PHASE1_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "controller/mtc.hpp"

class Phase1Node : public rclcpp::Node, public MTC {
public:
    Phase1Node(const std::string & name, const rclcpp::NodeOptions & options)
    : rclcpp::Node(name, options), MTC(options) {
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
            auto stage = std::make_unique<mtc::stages::MoveRelative>("linear push", cartesian_planner);
            stage->setGroup(task.properties().get<std::string>("group"));
            stage->setIKFrame(task.properties().get<std::string>("eef"));
            geometry_msgs::msg::Vector3Stamped direction;
            direction.header.frame_id = task.properties().get<std::string>("eef");
            direction.vector.y = 1.0;
            stage->setDirection(direction);
            stage->setMinMaxDistance(0.095, 0.1);
            task.add(std::move(stage));
        }
        return task;
    }
};

#endif // PHASE1_NODE_HPP