#pragma once

#include <vector>
#include <cmath>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Matrix3x3.h"

/**
 * @brief 圆弧路径生成器
 * 
 * 生成末端执行器绕指定旋转轴旋转的圆弧路径waypoints。
 * 使用密集路径点来近似圆弧运动，确保笛卡尔空间的平滑过渡。
 */
class CircularPathGenerator {
public:
    CircularPathGenerator() = default;

    /**
     * @brief 设置旋转轴在局部坐标系（target frame）中的位置
     * @param x X偏移量（米）
     * @param y Y偏移量（米）
     * @param z Z偏移量（米）
     */
    void setRotationAxisOffset(double x, double y, double z) {
        axis_offset_x_ = x;
        axis_offset_y_ = y;
        axis_offset_z_ = z;
    }

    /**
     * @brief 设置旋转轴方向（在target frame的局部坐标系中）
     * @param axis 0=X轴, 1=Y轴, 2=Z轴
     */
    void setRotationAxisDirection(int axis) {
        axis_direction_ = axis;
    }

    /**
     * @brief 设置旋转角度
     * @param angle 弧度值，正值为逆时针，负值为顺时针
     */
    void setRotationAngle(double angle) {
        rotation_angle_ = angle;
    }

    /**
     * @brief 设置生成的路径点数量
     * @param count 路径点数量（建议20-100）
     */
    void setWaypointCount(int count) {
        waypoint_count_ = count;
    }

    /**
     * @brief 生成圆弧路径上的所有waypoints
     * 
     * @param current_pose 当前末端执行器位姿（world frame）
     * @param target_frame_transform target frame在world中的变换
     * @return std::vector<geometry_msgs::msg::PoseStamped> 圆弧路径上的密集waypoints
     * 
     * 算法说明：
     * 1. 将末端执行器位姿从world frame变换到target frame
     * 2. 计算末端到旋转轴的相对位置（旋转半径向量）
     * 3. 将旋转角度均匀分割为多个小角度
     * 4. 对每个小角度：
     *    a. 使用罗德里格斯公式绕旋转轴旋转位置
     *    b. 同时旋转末端的姿态
     *    c. 变换回world frame
     * 5. 返回所有waypoints
     */
    std::vector<geometry_msgs::msg::PoseStamped> generateWaypoints(
        const geometry_msgs::msg::PoseStamped& current_pose,
        const geometry_msgs::msg::TransformStamped& target_frame_transform) {
        
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        waypoints.reserve(waypoint_count_);

        // 获取target frame在world中的transform
        tf2::Transform world_to_target;
        tf2::fromMsg(target_frame_transform.transform, world_to_target);
        tf2::Transform target_to_world = world_to_target.inverse();

        // 当前末端位姿（world frame）
        tf2::Transform current_world;
        tf2::fromMsg(current_pose.pose, current_world);

        // 将当前末端位姿转换到target frame
        tf2::Transform current_in_target = target_to_world * current_world;

        // 旋转轴在target frame中的位置
        tf2::Vector3 axis_origin(axis_offset_x_, axis_offset_y_, axis_offset_z_);

        // 旋转轴方向（在target frame中）
        tf2::Vector3 axis_direction;
        switch (axis_direction_) {
            case 0: axis_direction = tf2::Vector3(1, 0, 0); break;  // X轴
            case 1: axis_direction = tf2::Vector3(0, 1, 0); break;  // Y轴
            case 2: axis_direction = tf2::Vector3(0, 0, 1); break;  // Z轴
            default: axis_direction = tf2::Vector3(1, 0, 0); break;
        }
        axis_direction.normalize();

        // 末端到旋转轴原点的向量
        tf2::Vector3 ee_pos_in_target = current_in_target.getOrigin();
        tf2::Vector3 radius_vector = ee_pos_in_target - axis_origin;

        // 末端当前姿态（四元数）
        tf2::Quaternion ee_orientation_in_target = current_in_target.getRotation();

        // 计算每一步的旋转角度
        double angle_step = rotation_angle_ / (waypoint_count_ - 1);

        for (int i = 0; i < waypoint_count_; ++i) {
            double current_angle = angle_step * i;

            // 使用罗德里格斯公式计算旋转后的位置
            tf2::Vector3 rotated_radius = rotateVectorAroundAxis(
                radius_vector, axis_direction, current_angle);
            tf2::Vector3 new_pos_in_target = axis_origin + rotated_radius;

            // 旋转姿态（绕同一轴）
            tf2::Quaternion rotation_quat;
            rotation_quat.setRotation(axis_direction, current_angle);
            tf2::Quaternion new_orientation_in_target = rotation_quat * ee_orientation_in_target;
            new_orientation_in_target.normalize();

            // 构建target frame中的新姿态
            tf2::Transform new_pose_in_target;
            new_pose_in_target.setOrigin(new_pos_in_target);
            new_pose_in_target.setRotation(new_orientation_in_target);

            // 转换回world frame
            tf2::Transform new_pose_in_world = world_to_target * new_pose_in_target;

            // 创建PoseStamped消息
            geometry_msgs::msg::PoseStamped waypoint;
            waypoint.header = current_pose.header;
            
            // tf2::Transform转换为geometry_msgs::Pose
            waypoint.pose.position.x = new_pose_in_world.getOrigin().x();
            waypoint.pose.position.y = new_pose_in_world.getOrigin().y();
            waypoint.pose.position.z = new_pose_in_world.getOrigin().z();
            waypoint.pose.orientation = tf2::toMsg(new_pose_in_world.getRotation());
            
            waypoints.push_back(waypoint);
        }

        return waypoints;
    }

    /**
     * @brief 获取旋转半径（用于验证）
     */
    double getRotationRadius(
        const geometry_msgs::msg::PoseStamped& current_pose,
        const geometry_msgs::msg::TransformStamped& target_frame_transform) {
        
        tf2::Transform world_to_target;
        tf2::fromMsg(target_frame_transform.transform, world_to_target);
        tf2::Transform target_to_world = world_to_target.inverse();

        tf2::Transform current_world;
        tf2::fromMsg(current_pose.pose, current_world);

        tf2::Transform current_in_target = target_to_world * current_world;

        tf2::Vector3 axis_origin(axis_offset_x_, axis_offset_y_, axis_offset_z_);
        tf2::Vector3 ee_pos = current_in_target.getOrigin();
        tf2::Vector3 radius_vector = ee_pos - axis_origin;

        // 投影到垂直于旋转轴的平面上计算实际半径
        tf2::Vector3 axis_direction;
        switch (axis_direction_) {
            case 0: axis_direction = tf2::Vector3(1, 0, 0); break;
            case 1: axis_direction = tf2::Vector3(0, 1, 0); break;
            case 2: axis_direction = tf2::Vector3(0, 0, 1); break;
            default: axis_direction = tf2::Vector3(1, 0, 0); break;
        }
        
        tf2::Vector3 projection = radius_vector - 
            (radius_vector.dot(axis_direction)) * axis_direction;
        
        return projection.length();
    }

private:
    /**
     * @brief 罗德里格斯旋转公式 - 绕任意轴旋转向量
     * @param v 要旋转的向量
     * @param k 旋转轴（单位向量）
     * @param theta 旋转角度（弧度）
     * @return 旋转后的向量
     */
    tf2::Vector3 rotateVectorAroundAxis(
        const tf2::Vector3& v, 
        const tf2::Vector3& k, 
        double theta) {
        
        // 罗德里格斯公式：v_rot = v*cos(θ) + (k×v)*sin(θ) + k*(k·v)*(1-cos(θ))
        double cos_theta = std::cos(theta);
        double sin_theta = std::sin(theta);
        
        tf2::Vector3 v_rot = v * cos_theta + 
                            k.cross(v) * sin_theta + 
                            k * k.dot(v) * (1.0 - cos_theta);
        
        return v_rot;
    }

    double axis_offset_x_{0.0};
    double axis_offset_y_{0.0};
    double axis_offset_z_{0.0};
    int axis_direction_{0};  // 0=X, 1=Y, 2=Z
    double rotation_angle_{0.0};
    int waypoint_count_{50};
};
