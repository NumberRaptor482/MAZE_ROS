from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory('cartographer'), 'config')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    return LaunchDescription([

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'gazebo_2d.lua'
            ],
        ),
    ])


