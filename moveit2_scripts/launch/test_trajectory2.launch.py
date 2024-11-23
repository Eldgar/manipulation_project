import os
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Get the package directory for your MoveIt config package
    moveit_config_package = get_package_share_directory('my_moveit_config')
    
    # Build the MoveIt configs
    moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()
    
    # Include the MoveGroup launch file
    # move_group_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(moveit_config_package, 'launch', 'move_group.launch.py')
    #     ),
    #     launch_arguments={
    #         'use_sim_time': 'true',
    #         'robot_description': moveit_config.robot_description['robot_description'],
    #         'robot_description_semantic': moveit_config.robot_description_semantic['robot_description_semantic'],
    #         'robot_description_kinematics': moveit_config.robot_description_kinematics['robot_description_kinematics'],
    #     }.items(),
    # )
    
    # # Controllers (if not already launched)
    # controller_manager_node = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[
    #         moveit_config.robot_description,
    #         os.path.join(moveit_config_package, 'config', 'ros_controllers.yaml'),
    #         {'use_sim_time': True},
    #     ],
    #     output='screen',
    # )
    
    # # Spawner nodes for controllers
    # joint_state_broadcaster_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    #     parameters=[{'use_sim_time': True}],
    # )

    # Your custom node
    test_trajectory_node = Node(
        name="test_trajectory2",
        package="moveit2_scripts",
        executable="test_trajectory2",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        # move_group_launch,
        # controller_manager_node,
        # joint_state_broadcaster_spawner,
        test_trajectory_node,
    ])
