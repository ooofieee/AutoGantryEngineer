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
        task.properties().set("eef", "link7");

        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(0.5);
        cartesian_planner->setMaxAccelerationScalingFactor(0.5);
        cartesian_planner->setStepSize(0.001);

        {
            auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
            task.add(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("move to preloading", sampling_planner);
            stage->setGroup(task.properties().get<std::string>("group"));
            stage->setIKFrame(task.properties().get<std::string>("eef"));

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
            tf2::Vector3 z_offset(0.0, 0.0, 0.1);
            tf2::Vector3 rotated_offset = tf2::quatRotate(q, z_offset);
            
            goal_pose.pose.position.x += rotated_offset.x();
            goal_pose.pose.position.y += rotated_offset.y();
            goal_pose.pose.position.z += rotated_offset.z();

            goal_pose.pose.orientation = target_tf.transform.rotation;
            
            stage->setGoal(goal_pose);
            task.add(std::move(stage));
        }

        {
            
        }
        

        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("linear down", cartesian_planner);
            stage->setGroup(task.properties().get<std::string>("group"));
            stage->setIKFrame(task.properties().get<std::string>("eef"));
            geometry_msgs::msg::Vector3Stamped direction;
            direction.header.frame_id = "cylinder_target_frame";
            direction.vector.z = -0.15;
            stage->setDirection(direction);
            stage->setMinMaxDistance(0.145, 0.15);
            task.add(std::move(stage));
        }
        
        return task;
    }
};

#endif // PHASE0_NODE_HPP