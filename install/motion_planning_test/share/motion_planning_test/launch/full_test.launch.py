from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 加载MoveIt配置
    moveit_config = MoveItConfigsBuilder(
        "gantry_robot", 
        package_name="gantry_robot_moveit_config"
    ).to_moveit_configs()
    
    # 包含demo.launch.py以启动整个MoveIt环境
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gantry_robot_moveit_config"),
            "/launch/demo.launch.py"
        ])
    )
    
    # 末端执行器对齐测试节点（延迟启动，等待MoveIt初始化）
    ee_alignment_test_node = TimerAction(
        period=5.0,  # 延迟5秒启动
        actions=[
            Node(
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
        ]
    )
    
    return LaunchDescription([
        demo_launch,
        ee_alignment_test_node
    ])
