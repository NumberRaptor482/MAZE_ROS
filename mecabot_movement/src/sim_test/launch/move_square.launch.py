from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_test',
            executable='move_in_square', # This is the new executable we will define
            name='mecabot_square_node',
            output='screen'
        ),
    ])
