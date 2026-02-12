from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 获取MoveIt配置
    moveit_config = MoveItConfigsBuilder("gantry_robot", package_name="gantry_robot_moveit_config").to_moveit_configs()
    
    # 参数声明
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'planning_group',
            default_value='stand',
            description='MoveIt planning group name'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'reference_frame',
            default_value='ipm',
            description='Reference frame for random poses'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'end_effector',
            default_value='cylinder3',
            description='End effector link name'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'test_count',
            default_value='10',
            description='Number of random poses to test'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'planning_time',
            default_value='10.0',
            description='Planning time limit in seconds'
        )
    )
    
    # Stand随机位姿测试节点
    stand_random_pose_node = Node(
        package='motion_planning_test',
        executable='stand_random_pose',
        name='stand_random_pose_node',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {
                'planning_group': LaunchConfiguration('planning_group'),
                'reference_frame': LaunchConfiguration('reference_frame'),
                'end_effector_link': LaunchConfiguration('end_effector'),
                'test_count': LaunchConfiguration('test_count'),
                'planning_time': LaunchConfiguration('planning_time'),
            }
        ]
    )
    
    return LaunchDescription(declared_arguments + [stand_random_pose_node])
