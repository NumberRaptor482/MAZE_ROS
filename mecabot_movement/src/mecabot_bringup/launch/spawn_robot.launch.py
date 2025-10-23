import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get the path to the URDF file
    pkg_share = get_package_share_directory('mecabot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'mecabot.urdf.xacro')

    # Process the xacro file to generate the URDF
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # Create the node to spawn the robot
    # THIS IS THE CORRECTED NODE FOR ROS 2 GALACTIC
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description', # Spawn from this topic
            '-entity', 'mecabot'          # Name of the robot in Gazebo
        ],
        output='screen'
    )

    # A node to publish the robot description to a topic
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )


    return LaunchDescription([
        robot_state_publisher_node,
        spawn_entity_node
    ])
