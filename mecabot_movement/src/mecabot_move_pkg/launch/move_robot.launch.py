from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecabot_move_pkg',
            executable='move_robot',
            name='mecabot_move_node',
            output='screen'
        ),
    ])
