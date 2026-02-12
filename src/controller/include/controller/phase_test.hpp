#ifndef CONTROLLER_PHASE_TEST_HPP
#define CONTROLLER_PHASE_TEST_HPP

#include "controller/mtc.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class PhaseTestNode : public rclcpp::Node, public MTC {
public:
    PhaseTestNode(const std::string & name, const rclcpp::NodeOptions & options)
    : rclcpp::Node(name, options), MTC(options) {
        doTask();
    }

private:

    tf2_ros::Buffer tf_buffer_{this->get_clock()};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{std::make_shared<tf2_ros::TransformListener>(tf_buffer_)};

    void setup_planning_scene() override{}

    geometry_msgs::msg::TransformStamped get_tf(){
        try{
            return tf_buffer_.lookupTransform(
                "world", "cylinder_target_frame", tf2::TimePointZero);

        } catch(const std::exception & e){
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", e.what());
            return geometry_msgs::msg::TransformStamped();
        }
    }

    mtc::Task create_task() override{
        mtc::Task task = TaskInit("push");

        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(MTC::getNodeSharedPtr());
        sampling_planner->setMaxAccelerationScalingFactor(0.5);
        sampling_planner->setMaxVelocityScalingFactor(0.5);

        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(0.1);
        cartesian_planner->setMaxAccelerationScalingFactor(0.1);
        cartesian_planner->setStepSize(0.01);

        mtc::Stage* current_state_ptr = nullptr;
        {
            auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
            current_state_ptr = stage.get();
            task.add(std::move(stage));
        }

        {
            mtc::stages::Connect::GroupPlannerVector planners = {
                {"gantry_robot", sampling_planner}
            };
            auto stage = std::make_unique<mtc::stages::Connect>("move to preload", planners);
            stage->setTimeout(10.0);
            task.add(std::move(stage));
        }

        {   
            auto generator = std::make_unique<FuzzyPoseGenerator>("fuzzy pose generator");
            generator->setSampleCount(50);
            generator->setTolerance(0.002, 0.1);

            geometry_msgs::msg::TransformStamped target_tf = get_tf();
            
            geometry_msgs::msg::PoseStamped goal_pose;
            goal_pose.header.frame_id = "world";
            goal_pose.header.stamp = this->now();
            
            goal_pose.pose.position.x = target_tf.transform.translation.x;
            goal_pose.pose.position.y = target_tf.transform.translation.y;
            goal_pose.pose.position.z = target_tf.transform.translation.z;
            goal_pose.pose.orientation = target_tf.transform.rotation;

            tf2::Quaternion q(
                target_tf.transform.rotation.x,
                target_tf.transform.rotation.y,
                target_tf.transform.rotation.z,
                target_tf.transform.rotation.w
            );
            tf2::Vector3 z_offset(0.0, 0.0, 0.03);
            tf2::Vector3 rotated_offset = tf2::quatRotate(q, z_offset);
            
            goal_pose.pose.position.x += rotated_offset.x();
            goal_pose.pose.position.y += rotated_offset.y();
            goal_pose.pose.position.z += rotated_offset.z();

            goal_pose.pose.orientation = target_tf.transform.rotation;

            generator->setPose(goal_pose);
            generator->setMonitoredStage(current_state_ptr);
            generator->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("compute ik", std::move(generator));
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame("link7");
            wrapper->setTargetPose(goal_pose);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            wrapper->setIgnoreCollisions(true);
            task.add(std::move(wrapper));
        }

        return task;
    }
};

#endif // CONTROLLER_PHASE_TEST_HPP