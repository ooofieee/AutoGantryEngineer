#ifndef PHASE2_NODE_HPP
#define PHASE2_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "controller/mtc.hpp"
#include "controller/circular_path_generator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "moveit/robot_state/robot_state.h"

/**
 * @brief Phase 2 - 圆弧旋转阶段
 * 
 * 功能：以P轴为旋转轴旋转-90度
 * - P轴与target scene的x轴平行且正方向一致
 * - P轴相对于target frame: y偏移+108mm, z偏移-169mm
 * - 旋转角度: -90度（顺时针）
 * 
 * 实现方案：
 * 使用CircularPathGenerator生成密集圆弧路径点，
 * 然后通过MTC的Cartesian planner依次规划执行。
 */
class Phase2Node : public rclcpp::Node, public MTC {
public:
    Phase2Node(const std::string & name, const rclcpp::NodeOptions & options)
    : rclcpp::Node(name, options), MTC(options, name + "_mtc"),
      tf_buffer_{this->get_clock()},
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_)) {
        
        // 等待TF可用
        RCLCPP_INFO(this->get_logger(), "Phase2: Waiting for TF...");
        while (rclcpp::ok()) {
            try {
                tf_buffer_.lookupTransform("world", "link7", tf2::TimePointZero, std::chrono::seconds(1));
                tf_buffer_.lookupTransform("world", "cylinder_target_frame", tf2::TimePointZero, std::chrono::seconds(1));
                RCLCPP_INFO(this->get_logger(), "Phase2: TF available, starting circular motion task");
                break;
            } catch (const tf2::TransformException & e) {
                RCLCPP_WARN(this->get_logger(), "Waiting for transform: %s", e.what());
            }
        }
        
        // 初始化圆弧路径生成器
        initCircularPathGenerator();
        
        doTask();
    }

private:
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    CircularPathGenerator path_generator_;
    
    // 圆弧参数常量
    static constexpr double P_AXIS_Y_OFFSET = 0.108;   // y偏移+108mm
    static constexpr double P_AXIS_Z_OFFSET = -0.169;  // z偏移-169mm
    static constexpr double ROTATION_ANGLE = -M_PI_2; // -90度
    static constexpr int WAYPOINT_COUNT = 50;          // 圆弧路径点数量

    void initCircularPathGenerator() {
        // P轴在target frame中的位置: (0, +108mm, -169mm)
        path_generator_.setRotationAxisOffset(0.0, P_AXIS_Y_OFFSET, P_AXIS_Z_OFFSET);
        
        // P轴与target frame的x轴平行
        path_generator_.setRotationAxisDirection(0);  // 0 = X轴
        
        // 旋转-90度
        path_generator_.setRotationAngle(ROTATION_ANGLE);
        
        // 50个路径点，确保圆弧平滑
        path_generator_.setWaypointCount(WAYPOINT_COUNT);
    }

    geometry_msgs::msg::TransformStamped getTargetFrameTransform() {
        try {
            return tf_buffer_.lookupTransform(
                "world", "cylinder_target_frame", tf2::TimePointZero);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Could not get target frame transform: %s", e.what());
            throw;
        }
    }

    geometry_msgs::msg::PoseStamped getCurrentEndEffectorPose() {
        try {
            auto tf = tf_buffer_.lookupTransform("world", "link7", tf2::TimePointZero);
            
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.header.stamp = this->now();
            pose.pose.position.x = tf.transform.translation.x;
            pose.pose.position.y = tf.transform.translation.y;
            pose.pose.position.z = tf.transform.translation.z;
            pose.pose.orientation = tf.transform.rotation;
            
            return pose;
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Could not get end effector pose: %s", e.what());
            throw;
        }
    }

    void setup_planning_scene() override {
        // Phase2不需要额外的场景设置
    }

    mtc::Task create_task() override {
        mtc::Task task = TaskInit("circular_rotation");
        
        // Cartesian planner用于圆弧路径（每个小段近似为直线）
        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(0.1);
        cartesian_planner->setMaxAccelerationScalingFactor(0.1);
        cartesian_planner->setStepSize(0.005);  // 更小的步长确保精度
        
        // Sampling planner用于移动到中间waypoints
        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(MTC::getNodeSharedPtr());
        sampling_planner->setMaxVelocityScalingFactor(0.2);
        sampling_planner->setMaxAccelerationScalingFactor(0.2);

        mtc::Stage* current_state_ptr = nullptr;
        
        // Stage 1: 获取当前状态
        {
            auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
            current_state_ptr = stage.get();
            task.add(std::move(stage));
        }

        // 获取当前末端位姿和target frame变换
        geometry_msgs::msg::PoseStamped current_pose = getCurrentEndEffectorPose();
        geometry_msgs::msg::TransformStamped target_tf = getTargetFrameTransform();

        // 生成圆弧路径waypoints
        auto waypoints = path_generator_.generateWaypoints(current_pose, target_tf);
        
        double radius = path_generator_.getRotationRadius(current_pose, target_tf);
        RCLCPP_INFO(this->get_logger(), 
            "Phase2: Generated %zu waypoints for circular motion (radius: %.3f m, angle: %.1f deg)",
            waypoints.size(), radius, ROTATION_ANGLE * 180.0 / M_PI);

        // Stage 2: 使用多个MoveRelative stages实现圆弧运动
        // 策略：将圆弧分割为多段，每段使用Cartesian planner
        // 这样可以保证笛卡尔空间的平滑性

        const int segments = 10;  // 将圆弧分为10段
        const int points_per_segment = WAYPOINT_COUNT / segments;

        for (int seg = 0; seg < segments; ++seg) {
            int target_idx = (seg + 1) * points_per_segment - 1;
            if (target_idx >= static_cast<int>(waypoints.size())) {
                target_idx = waypoints.size() - 1;
            }

            const auto& target_pose = waypoints[target_idx];
            
            // 使用Connect stage连接到下一个关键waypoint
            {
                mtc::stages::Connect::GroupPlannerVector planners = {
                    {"gantry_robot", cartesian_planner}
                };
                
                auto stage = std::make_unique<mtc::stages::Connect>(
                    "arc_segment_" + std::to_string(seg), planners);
                stage->setTimeout(10.0);
                
                // 添加路径约束确保末端姿态连续变化
                moveit_msgs::msg::Constraints path_constraints;
                
                moveit_msgs::msg::OrientationConstraint ori_constraint;
                ori_constraint.link_name = "link7";
                ori_constraint.header.frame_id = "world";
                ori_constraint.orientation = target_pose.pose.orientation;
                ori_constraint.absolute_x_axis_tolerance = 0.5;
                ori_constraint.absolute_y_axis_tolerance = 0.5;
                ori_constraint.absolute_z_axis_tolerance = 0.5;
                ori_constraint.weight = 1.0;
                
                path_constraints.orientation_constraints.push_back(ori_constraint);
                stage->setPathConstraints(path_constraints);
                
                task.add(std::move(stage));
            }

            // 使用目标位姿作为下一个阶段的目标
            {
                auto generator = std::make_unique<mtc::stages::GeneratePose>("target_pose_" + std::to_string(seg));
                generator->setPose(target_pose);
                generator->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                
                auto wrapper = std::make_unique<mtc::stages::ComputeIK>(
                    "compute_ik_" + std::to_string(seg), std::move(generator));
                wrapper->setMaxIKSolutions(8);
                wrapper->setMinSolutionDistance(0.5);
                wrapper->setIKFrame("link7");
                wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
                wrapper->setIgnoreCollisions(false);
                
                task.add(std::move(wrapper));
            }
        }

        RCLCPP_INFO(this->get_logger(), "Phase2: Task created with %d arc segments", segments);
        
        return task;
    }
};

#endif // PHASE2_NODE_HPP