from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 加载MoveIt配置
    moveit_config = MoveItConfigsBuilder(
        "gantry_robot", 
        package_name="gantry_robot_moveit_config"
    ).to_moveit_configs()
    
    # 末端执行器对齐测试节点
    ee_alignment_test_node = Node(
        package="motion_planning_test",
        executable="ee_alignment_test",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )
    
    return LaunchDescription([
        ee_alignment_test_node
    ])
