#ifndef PHASE0_NODE_HPP
#define PHASE0_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "controller/mtc.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"

class Phase0Node : public rclcpp::Node, public MTC {
public:
    Phase0Node(const std::string & name, const rclcpp::NodeOptions & options)
    : rclcpp::Node(name, options), MTC(options),
      tf_buffer_{this->get_clock()},
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_)) {
        RCLCPP_INFO(this->get_logger(), "Waiting for TF...");
        while (rclcpp::ok()) {
            try {
                tf_buffer_.lookupTransform("world", "link7", tf2::TimePointZero, std::chrono::seconds(1));
                RCLCPP_INFO(this->get_logger(), "TF available, starting task");
                break;
            } catch (const tf2::TransformException & e) {
                RCLCPP_WARN(this->get_logger(), "Waiting for transform: %s", e.what());
            }
        }
        doTask();
      }

private:

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


    geometry_msgs::msg::TransformStamped get_tf(){
        try{
            return tf_buffer_.lookupTransform(
                "world", "cylinder_target_frame", tf2::TimePointZero);

        } catch(const std::exception & e){
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", e.what());
            return geometry_msgs::msg::TransformStamped();
        }
    }

    void setup_planning_scene() override{

    }

    mtc::Task create_task() override{
        mtc::Task task = TaskInit("load");
        
        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(MTC::getNodeSharedPtr());
        sampling_planner->setMaxAccelerationScalingFactor(0.5);
        sampling_planner->setMaxVelocityScalingFactor(0.5);
        sampling_planner->setProperty("planning_time", 5.0);

        task.properties().set("group", "gantry_robot");
        task.properties().set("ik_frame", "link7");

        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(0.5);
        cartesian_planner->setMaxAccelerationScalingFactor(0.5);
        cartesian_planner->setStepSize(0.0001);

        mtc::Stage* current_state_ptr = nullptr;
        mtc::Stage* preload_state_ptr = nullptr;
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
            auto generator = std::make_unique<FuzzyPoseGenerator>("fuzzy pose generator for preloading");
            generator->setSampleCount(50);
            generator->setTolerance(0.03, 0.3);

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

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("compute ik for preloading", std::move(generator));
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame("link7");
            wrapper->setTargetPose(goal_pose);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            wrapper->setIgnoreCollisions(true);
            preload_state_ptr = wrapper.get();
            task.add(std::move(wrapper));
        }
        
        // {
        //     auto stage = std::make_unique<mtc::stages::MoveRelative>("linear down", cartesian_planner);
        //     stage->setGroup(task.properties().get<std::string>("group"));
        //     stage->setIKFrame("link7");
        //     geometry_msgs::msg::Vector3Stamped direction;
        //     direction.header.frame_id = "cylinder_target_frame";
        //     direction.vector.z = -0.15;
        //     stage->setDirection(direction);
        //     stage->setMinMaxDistance(0.145, 0.15);
        //     task.add(std::move(stage));
        // }

        {
            mtc::stages::Connect::GroupPlannerVector planners = {
                {"gantry_robot", sampling_planner}
            };

            auto stage = std::make_unique<mtc::stages::Connect>("move to load", planners);
            stage->setTimeout(10.0);

            moveit_msgs::msg::Constraints path_constraints;

            moveit_msgs::msg::OrientationConstraint ori_constraint;
            ori_constraint.link_name = "link7";
            ori_constraint.header.frame_id = "world";
            ori_constraint.orientation.w = 1.0;
            ori_constraint.absolute_x_axis_tolerance = 0.5;
            ori_constraint.absolute_y_axis_tolerance = 0.5;
            ori_constraint.absolute_z_axis_tolerance = 3.14;

            moveit_msgs::msg::PositionConstraint pos_constraint;
            pos_constraint.link_name = "link7";
            pos_constraint.header.frame_id = "world";
            shape_msgs::msg::SolidPrimitive bounding_region;
            bounding_region.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
            bounding_region.dimensions = {0.2, 0.05};
            pos_constraint.constraint_region.primitives.push_back(bounding_region);
            
            geometry_msgs::msg::TransformStamped target_tf = get_tf();
            geometry_msgs::msg::PoseStamped goal_pose;
            goal_pose.header.frame_id = "world";
            goal_pose.header.stamp = this->now();
            goal_pose.pose.position.x = target_tf.transform.translation.x;
            goal_pose.pose.position.y = target_tf.transform.translation.y;
            goal_pose.pose.position.z = target_tf.transform.translation.z;
            goal_pose.pose.orientation = target_tf.transform.rotation;
            pos_constraint.constraint_region.primitive_poses.push_back(goal_pose.pose);

            path_constraints.orientation_constraints.push_back(ori_constraint);
            path_constraints.position_constraints.push_back(pos_constraint);
            stage->setPathConstraints(path_constraints);

            task.add(std::move(stage));
        }

        {
            auto generator = std::make_unique<FuzzyPoseGenerator>("fuzzy pose generator for loading");
            generator->setSampleCount(50);
            generator->setTolerance(0.002, 0.1);

            geometry_msgs::msg::TransformStamped target_tf = get_tf();
            generator->setMonitoredStage(preload_state_ptr);
            geometry_msgs::msg::PoseStamped goal_pose;
            goal_pose.header.frame_id = "world";
            goal_pose.header.stamp = this->now();
            goal_pose.pose.position.x = target_tf.transform.translation.x;
            goal_pose.pose.position.y = target_tf.transform.translation.y;
            goal_pose.pose.position.z = target_tf.transform.translation.z;
            goal_pose.pose.orientation = target_tf.transform.rotation;

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
            tf2::Vector3 z_offset(0.0, 0.0, -0.15);
            tf2::Vector3 rotated_offset = tf2::quatRotate(q, z_offset);
            
            goal_pose.pose.position.x += rotated_offset.x();
            goal_pose.pose.position.y += rotated_offset.y();
            goal_pose.pose.position.z += rotated_offset.z();

            goal_pose.pose.orientation = target_tf.transform.rotation;

            generator->setPose(goal_pose);
            generator->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("compute ik for loading", std::move(generator));
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame("link7");
            wrapper->setTargetPose(goal_pose);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            wrapper->setIgnoreCollisions(true);
            task.add(std::move(wrapper));
        }

        // {
        //     auto stage = std::make_unique<mtc::stages::MoveTo>("move back to home", sampling_planner);
        //     stage->setTimeout(10.0);
        //     stage->setGoal("home_gr");
        //     stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        //     task.add(std::move(stage));
        // }
        
        return task;
    }
};

#endif // PHASE0_NODE_HPP