#ifndef PHASE1_NODE_HPP
#define PHASE1_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "controller/mtc.hpp"

class Phase2Node : public rclcpp::Node, public MTC {
public:
    Phase2Node(const std::string & name, const rclcpp::NodeOptions & options)
    : rclcpp::Node(name, options), MTC(options, name + "_mtc") {}

private:

    void setup_planning_scene() override{}

    mtc::Task create_task() override{
        mtc::Task task;
        task.stages()->setName("push");
        task.loadRobotModel(MTC::getNodeSharedPtr());

        const auto& arm_group = "gantry_robot";
        const auto& hand_group = "gantry_hand";
        const auto& hand_frame = "link7";

        task.setProperty("group", arm_group);        
        task.setProperty("hand_group", hand_group);
        task.setProperty("eef", hand_frame);

        mtc::Stage* current_state_ptr = nullptr;
        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state_ptr = stage_state_current.get();
        task.add(std::move(stage_state_current));

        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(MTC::getNodeSharedPtr());
        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(0.1);
        cartesian_planner->setMaxAccelerationScalingFactor(0.1);
        cartesian_planner->setStepSize(0.01);

        {
            
        }
        return task;
    }
};

#endif // PHASE1_NODE_HPP