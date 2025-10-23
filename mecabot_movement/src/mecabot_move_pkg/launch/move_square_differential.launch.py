from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecabot_move_pkg',
            executable='move_square_differential', # The new executable
            name='mecabot_square_differential_node',
            output='screen'
        ),
    ])
