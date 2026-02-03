#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class StandRandomPoseNode : public rclcpp::Node
{
public:
    StandRandomPoseNode() 
    : Node("stand_random_pose_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
    {
        // 声明参数
        this->declare_parameter("planning_group", "stand");
        this->declare_parameter("reference_frame", "ipm");
        this->declare_parameter("end_effector_link", "cylinder3");
        this->declare_parameter("test_count", 10);
        this->declare_parameter("planning_time", 10.0);
        
        // 获取参数
        planning_group_ = this->get_parameter("planning_group").as_string();
        reference_frame_ = this->get_parameter("reference_frame").as_string();
        end_effector_link_ = this->get_parameter("end_effector_link").as_string();
        test_count_ = this->get_parameter("test_count").as_int();
        planning_time_ = this->get_parameter("planning_time").as_double();
        
        // 初始化随机数生成器
        rng_.seed(std::random_device{}());
        
        RCLCPP_INFO(this->get_logger(), "Stand Random Pose Node initialized");
        RCLCPP_INFO(this->get_logger(), "Planning group: %s", planning_group_.c_str());
        RCLCPP_INFO(this->get_logger(), "Reference frame: %s", reference_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "End effector: %s", end_effector_link_.c_str());
        
        // 等待TF可用
        waitForTransform();
        
        // 创建定时器延迟启动
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&StandRandomPoseNode::runTests, this));
    }

private:
    void waitForTransform()
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for transform from world to %s...", reference_frame_.c_str());
        while (rclcpp::ok()) {
            try {
                tf_buffer_.lookupTransform("world", reference_frame_, tf2::TimePointZero, std::chrono::seconds(1));
                RCLCPP_INFO(this->get_logger(), "Transform available!");
                break;
            } catch (const tf2::TransformException & e) {
                RCLCPP_WARN(this->get_logger(), "Waiting for transform: %s", e.what());
            }
        }
    }

    geometry_msgs::msg::PoseStamped generateRandomPose()
    {
        // 定义随机范围（单位：米和弧度）- 非常保守的范围
        std::uniform_real_distribution<double> x_dist(-0.02, 0.02);   // ±20mm
        std::uniform_real_distribution<double> y_dist(-0.02, 0.02);   // ±20mm
        std::uniform_real_distribution<double> z_dist(-0.03, -0.01);  // -30mm到-10mm（略微向下）
        
        // 姿态范围 - 非常小的角度变化
        std::uniform_real_distribution<double> roll_dist(-0.1745, 0.1745);   // ±10°
        std::uniform_real_distribution<double> pitch_dist(-0.2618, 0.0);     // -15° to 0°
        std::uniform_real_distribution<double> yaw_dist(-0.1745, 0.1745);    // ±10°
        
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = reference_frame_;
        target_pose.header.stamp = this->now();
        
        // 生成随机位置
        target_pose.pose.position.x = x_dist(rng_);
        target_pose.pose.position.y = y_dist(rng_);
        target_pose.pose.position.z = z_dist(rng_);
        
        // 生成随机姿态（RPY转四元数）
        double roll = roll_dist(rng_);
        double pitch = pitch_dist(rng_);
        double yaw = yaw_dist(rng_);
        
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        target_pose.pose.orientation = tf2::toMsg(q);
        
        RCLCPP_INFO(this->get_logger(), 
            "Generated random pose in %s: pos[%.3f, %.3f, %.3f] rpy[%.2f°, %.2f°, %.2f°]",
            reference_frame_.c_str(),
            target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z,
            roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
        
        return target_pose;
    }

    void runTests()
    {
        // 停止定时器，只运行一次
        timer_->cancel();
        
        // 创建MoveGroupInterface（使用当前节点以访问参数）
        RCLCPP_INFO(this->get_logger(), "Creating MoveGroupInterface...");
        auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), planning_group_);
        
        // 设置规划参数
        move_group->setPlanningTime(planning_time_);
        move_group->setNumPlanningAttempts(20);  // 增加尝试次数
        move_group->setMaxVelocityScalingFactor(0.1);  // 降低速度以提高成功率
        move_group->setMaxAccelerationScalingFactor(0.1);
        move_group->setEndEffectorLink(end_effector_link_);
        move_group->allowReplanning(true);  // 允许重新规划
        move_group->setGoalTolerance(0.01);  // 设置目标容差
        
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group->getEndEffectorLink().c_str());
        RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group->getPlanningFrame().c_str());
        
        // 打印当前位姿
        try {
            auto current_pose = move_group->getCurrentPose(end_effector_link_);
            RCLCPP_INFO(this->get_logger(), "Current pose of %s: [%.3f, %.3f, %.3f]",
                end_effector_link_.c_str(),
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Could not get current pose: %s", e.what());
        }
        
        // 运行测试
        int success_count = 0;
        for (int i = 0; i < test_count_; ++i) {
            RCLCPP_INFO(this->get_logger(), "\n========== Test %d/%d ==========", i + 1, test_count_);
            
            // 生成随机目标位姿
            auto target_pose = generateRandomPose();
            
            // 转换到world坐标系（MoveIt的planning frame）
            geometry_msgs::msg::PoseStamped target_pose_world;
            try {
                auto transform = tf_buffer_.lookupTransform(
                    "world", 
                    target_pose.header.frame_id,
                    tf2::TimePointZero,
                    std::chrono::seconds(1));
                tf2::doTransform(target_pose, target_pose_world, transform);
                
                RCLCPP_INFO(this->get_logger(), 
                    "Transformed to world: pos[%.3f, %.3f, %.3f]",
                    target_pose_world.pose.position.x,
                    target_pose_world.pose.position.y,
                    target_pose_world.pose.position.z);
            } catch (const tf2::TransformException& e) {
                RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", e.what());
                continue;
            }
            
            // 设置目标位姿
            move_group->setPoseTarget(target_pose_world, end_effector_link_);
            
            // 规划
            RCLCPP_INFO(this->get_logger(), "Planning...");
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto result = move_group->plan(plan);
            
            if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "✓ Planning succeeded!");
                
                // 执行
                RCLCPP_INFO(this->get_logger(), "Executing...");
                auto execute_result = move_group->execute(plan);
                
                if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "✓ Execution succeeded!");
                    success_count++;
                    
                    // 等待一段时间
                    rclcpp::sleep_for(std::chrono::seconds(1));
                } else {
                    RCLCPP_ERROR(this->get_logger(), "✗ Execution failed!");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "✗ Planning failed! Error code: %d", result.val);
                
                // 添加更多诊断信息
                auto joint_values = move_group->getCurrentJointValues();
                RCLCPP_INFO(this->get_logger(), "  Current joint count: %zu", joint_values.size());
                
                // 尝试使用随机有效状态作为目标
                RCLCPP_INFO(this->get_logger(), "  Trying to find if pose is reachable...");
                move_group->setRandomTarget();
                auto random_result = move_group->plan(plan);
                if (random_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "  → Random planning works, so planner is functional");
                } else {
                    RCLCPP_WARN(this->get_logger(), "  → Even random planning fails");
                }
            }
            
            // 清除目标
            move_group->clearPoseTargets();
        }
        
        // 总结
        RCLCPP_INFO(this->get_logger(), "\n========== Test Summary ==========");
        RCLCPP_INFO(this->get_logger(), "Total tests: %d", test_count_);
        RCLCPP_INFO(this->get_logger(), "Successful: %d (%.1f%%)", 
            success_count, 100.0 * success_count / test_count_);
        RCLCPP_INFO(this->get_logger(), "Failed: %d", test_count_ - success_count);
        
        RCLCPP_INFO(this->get_logger(), "All tests completed!");
    }

    // 成员变量
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mt19937 rng_;
    
    // 参数
    std::string planning_group_;
    std::string reference_frame_;
    std::string end_effector_link_;
    int test_count_;
    double planning_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<StandRandomPoseNode>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
