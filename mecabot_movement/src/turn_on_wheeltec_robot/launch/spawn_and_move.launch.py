import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_mecabot_move = get_package_share_directory('mecabot_move_pkg')

    # Node to spawn the robot entity in Gazebo
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-topic', 'robot_description', '-entity', 'mecabot'],
        output='screen'
    )

    # Launch your movement controller
    move_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mecabot_move, 'launch', 'move_robot.launch.py')
        )
    )

    return LaunchDescription([
        spawn_entity,
        move_robot_launch,
    ])
