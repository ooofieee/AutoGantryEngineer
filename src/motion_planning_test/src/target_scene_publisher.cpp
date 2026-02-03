/**
 * @file target_scene_publisher.cpp
 * @brief 发布 planning_scene，包含一个以 ipm 为参考系随机位姿的 cylinder target
 * 
 * Target 参数:
 * - 形状: cylinder (半径 0.018m, 长度 0.1m)
 * - 坐标原点: 底面中心（作为正面）
 * 
 * 随机位姿范围（相对于 ipm 坐标系）:
 * - 位置: x: [-0.1, 0.1]m, y: [-0.1, 0.1]m, z: [-0.1, 0]m
 * - 姿态: roll: [-45, 45]°, pitch: [-90, 0]°, yaw: [-90, 90]°
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <random>
#include <chrono>

class TargetScenePublisher : public rclcpp::Node
{
public:
    TargetScenePublisher() : Node("target_scene_publisher")
    {
        // 声明参数
        this->declare_parameter("reference_frame", "ipm");
        this->declare_parameter("target_name", "cylinder_target");
        this->declare_parameter("cylinder_radius", 0.018);
        this->declare_parameter("cylinder_length", 0.1);
        this->declare_parameter("auto_publish", true);
        this->declare_parameter("publish_interval", 5.0);  // 秒
        
        // 位置范围参数 (单位: mm，内部转换为 m)
        this->declare_parameter("x_min", -100.0);
        this->declare_parameter("x_max", 100.0);
        this->declare_parameter("y_min", -100.0);
        this->declare_parameter("y_max", 100.0);
        this->declare_parameter("z_min", -100.0);
        this->declare_parameter("z_max", 0.0);
        
        // 姿态范围参数 (单位: 度)
        this->declare_parameter("roll_min", -45.0);
        this->declare_parameter("roll_max", 45.0);
        this->declare_parameter("pitch_min", -90.0);
        this->declare_parameter("pitch_max", 0.0);
        this->declare_parameter("yaw_min", -90.0);
        this->declare_parameter("yaw_max", 90.0);
        
        // 获取参数
        reference_frame_ = this->get_parameter("reference_frame").as_string();
        target_name_ = this->get_parameter("target_name").as_string();
        cylinder_radius_ = this->get_parameter("cylinder_radius").as_double();
        cylinder_length_ = this->get_parameter("cylinder_length").as_double();
        auto_publish_ = this->get_parameter("auto_publish").as_bool();
        publish_interval_ = this->get_parameter("publish_interval").as_double();
        
        // 位置范围 (mm -> m)
        x_min_ = this->get_parameter("x_min").as_double() / 1000.0;
        x_max_ = this->get_parameter("x_max").as_double() / 1000.0;
        y_min_ = this->get_parameter("y_min").as_double() / 1000.0;
        y_max_ = this->get_parameter("y_max").as_double() / 1000.0;
        z_min_ = this->get_parameter("z_min").as_double() / 1000.0;
        z_max_ = this->get_parameter("z_max").as_double() / 1000.0;
        
        // 姿态范围 (度 -> 弧度)
        roll_min_ = this->get_parameter("roll_min").as_double() * M_PI / 180.0;
        roll_max_ = this->get_parameter("roll_max").as_double() * M_PI / 180.0;
        pitch_min_ = this->get_parameter("pitch_min").as_double() * M_PI / 180.0;
        pitch_max_ = this->get_parameter("pitch_max").as_double() * M_PI / 180.0;
        yaw_min_ = this->get_parameter("yaw_min").as_double() * M_PI / 180.0;
        yaw_max_ = this->get_parameter("yaw_max").as_double() * M_PI / 180.0;
        
        // 初始化随机数生成器
        std::random_device rd;
        rng_ = std::mt19937(rd());
        
        // TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // Planning scene publisher
        planning_scene_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
            "planning_scene", 10);
        
        // 服务：手动触发生成新的随机 target
        generate_target_service_ = this->create_service<std_srvs::srv::Trigger>(
            "generate_random_target",
            std::bind(&TargetScenePublisher::generateTargetCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Target Scene Publisher 初始化完成");
        RCLCPP_INFO(this->get_logger(), "参考坐标系: %s", reference_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Cylinder: 半径=%.3fm, 长度=%.3fm", cylinder_radius_, cylinder_length_);
        RCLCPP_INFO(this->get_logger(), "位置范围 (m): x[%.3f, %.3f], y[%.3f, %.3f], z[%.3f, %.3f]",
                    x_min_, x_max_, y_min_, y_max_, z_min_, z_max_);
        RCLCPP_INFO(this->get_logger(), "姿态范围 (deg): roll[%.1f, %.1f], pitch[%.1f, %.1f], yaw[%.1f, %.1f]",
                    roll_min_ * 180.0 / M_PI, roll_max_ * 180.0 / M_PI,
                    pitch_min_ * 180.0 / M_PI, pitch_max_ * 180.0 / M_PI,
                    yaw_min_ * 180.0 / M_PI, yaw_max_ * 180.0 / M_PI);
        
        // 延迟初始化，等待 MoveIt 和 TF 准备好
        init_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&TargetScenePublisher::initCallback, this));
    }

private:
    void initCallback()
    {
        init_timer_->cancel();
        
        // 发布第一个 target
        publishRandomTarget();
        
        // 启动 TF 发布定时器，每 50ms 发布一次（20Hz）
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TargetScenePublisher::publishTargetTFTimer, this));
        
        // 如果启用自动发布，创建定时器
        if (auto_publish_) {
            publish_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(publish_interval_),
                std::bind(&TargetScenePublisher::publishRandomTarget, this));
            RCLCPP_INFO(this->get_logger(), "自动发布已启用，间隔: %.1f 秒", publish_interval_);
        }
    }
    
    geometry_msgs::msg::Pose generateRandomPose()
    {
        geometry_msgs::msg::Pose pose;
        
        // 随机位置
        std::uniform_real_distribution<double> x_dist(x_min_, x_max_);
        std::uniform_real_distribution<double> y_dist(y_min_, y_max_);
        std::uniform_real_distribution<double> z_dist(z_min_, z_max_);
        
        pose.position.x = x_dist(rng_);
        pose.position.y = y_dist(rng_);
        pose.position.z = z_dist(rng_);
        
        // 随机姿态
        std::uniform_real_distribution<double> roll_dist(roll_min_, roll_max_);
        std::uniform_real_distribution<double> pitch_dist(pitch_min_, pitch_max_);
        std::uniform_real_distribution<double> yaw_dist(yaw_min_, yaw_max_);
        
        double roll = roll_dist(rng_);
        double pitch = pitch_dist(rng_);
        double yaw = yaw_dist(rng_);
        
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        pose.orientation = tf2::toMsg(q);
        
        RCLCPP_INFO(this->get_logger(), 
            "生成随机位姿: pos(%.3f, %.3f, %.3f)m, rpy(%.1f, %.1f, %.1f)deg",
            pose.position.x, pose.position.y, pose.position.z,
            roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
        
        return pose;
    }
    
    moveit_msgs::msg::CollisionObject createCylinderTarget(const geometry_msgs::msg::Pose& pose)
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = reference_frame_;
        collision_object.header.stamp = this->now();
        collision_object.id = target_name_;
        
        // 定义 cylinder 形状
        // 注意: MoveIt 的 cylinder 默认原点在中心，我们需要将其偏移使原点在底面中心
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[primitive.CYLINDER_HEIGHT] = cylinder_length_;
        primitive.dimensions[primitive.CYLINDER_RADIUS] = cylinder_radius_;
        
        // 调整位姿，使得坐标原点在底面中心
        // cylinder 默认原点在中心，需要沿 z 轴（cylinder 轴向）偏移 length/2
        geometry_msgs::msg::Pose adjusted_pose = pose;
        
        // 获取当前姿态的旋转矩阵
        tf2::Quaternion q;
        tf2::fromMsg(pose.orientation, q);
        tf2::Matrix3x3 rotation_matrix(q);
        
        // cylinder 在其局部坐标系中沿 z 轴，需要将中心偏移到底面
        // 偏移量为 -length/2，使底面在原点，z轴正方向从底面向外指
        tf2::Vector3 offset(0, 0, -cylinder_length_ / 2.0);
        tf2::Vector3 world_offset = rotation_matrix * offset;
        
        adjusted_pose.position.x += world_offset.x();
        adjusted_pose.position.y += world_offset.y();
        adjusted_pose.position.z += world_offset.z();
        
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(adjusted_pose);
        collision_object.operation = collision_object.ADD;
        
        return collision_object;
    }
    
    void publishRandomTarget()
    {
        // 生成随机位姿
        geometry_msgs::msg::Pose random_pose = generateRandomPose();
        
        // 创建 collision object
        moveit_msgs::msg::CollisionObject target = createCylinderTarget(random_pose);
        
        // 创建 planning scene 消息
        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.is_diff = true;
        planning_scene_msg.world.collision_objects.push_back(target);
        
        // 发布
        planning_scene_pub_->publish(planning_scene_msg);
        
        RCLCPP_INFO(this->get_logger(), "已发布 cylinder target '%s' 到 planning scene", 
                    target_name_.c_str());
        
        // 保存当前 target 位姿（相对于参考系的原始位姿，底面中心）
        current_target_pose_ = random_pose;
        
        // 发布 TF 以显示坐标轴
        publishTargetTF(random_pose);
    }
    
    void publishTargetTF(const geometry_msgs::msg::Pose& pose)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = reference_frame_;
        transform.child_frame_id = target_name_ + "_frame";
        
        transform.transform.translation.x = pose.position.x;
        transform.transform.translation.y = pose.position.y;
        transform.transform.translation.z = pose.position.z;
        transform.transform.rotation = pose.orientation;
        
        tf_broadcaster_->sendTransform(transform);
    }
    
    void publishTargetTFTimer()
    {
        // 定时器回调，持续发布当前 target 的 TF
        publishTargetTF(current_target_pose_);
    }
    
    void generateTargetCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        publishRandomTarget();
        response->success = true;
        response->message = "已生成新的随机 target";
    }
    
    void removeTarget()
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = reference_frame_;
        collision_object.header.stamp = this->now();
        collision_object.id = target_name_;
        collision_object.operation = collision_object.REMOVE;
        
        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.is_diff = true;
        planning_scene_msg.world.collision_objects.push_back(collision_object);
        
        planning_scene_pub_->publish(planning_scene_msg);
        
        RCLCPP_INFO(this->get_logger(), "已从 planning scene 中移除 '%s'", target_name_.c_str());
    }

    // 参数
    std::string reference_frame_;
    std::string target_name_;
    double cylinder_radius_;
    double cylinder_length_;
    bool auto_publish_;
    double publish_interval_;
    
    // 随机范围
    double x_min_, x_max_;
    double y_min_, y_max_;
    double z_min_, z_max_;
    double roll_min_, roll_max_;
    double pitch_min_, pitch_max_;
    double yaw_min_, yaw_max_;
    
    // 随机数生成器
    std::mt19937 rng_;
    
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Publishers
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
    
    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr generate_target_service_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    
    // 当前 target 位姿
    geometry_msgs::msg::Pose current_target_pose_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetScenePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
