from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("gantry_robot", package_name="gantry_robot_moveit_config_sim").to_moveit_configs()
    
    # Robot state publisher already publishes all TFs from URDF, no need for static_tf

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # Ros2 control node
    ros2_controllers_path = os.path.join(
        get_package_share_directory("gantry_robot_moveit_config_sim"),
        "config",
        "ros2_controllers.yaml",
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # Load controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    gantry_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gantry_robot_controller", "--controller-manager", "/controller_manager"],
    )

    # Move group with explicit capabilities parameter
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
            {"use_sim_time": True},
        ],
    )

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("gantry_robot_moveit_config_sim"),
        "config",
        "moveit.rviz"
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        gantry_robot_controller_spawner,
        move_group_node,
        rviz_node,
    ])
