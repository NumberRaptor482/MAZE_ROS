import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('turn_on_wheeltec_robot')
    
    xacro_file = os.path.join(pkg_path, 'description', 'mecabot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')]),
        launch_arguments={'ign_args': '-r -v 4 empty.sdf'}.items()
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # Simple, working bridge configuration
    # --- CORRECTED: Add the ROS-Ignition Bridge ---
    # This bridge directly maps the ROS 2 /cmd_vel topic to the Ignition topic for the robot.
    ign_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/mecabot/cmd_vel@geometry_msgs/msg/Twist' +
            ']' +
            'ignition.msgs.Twist'
        ],
        remappings=[
            ('/model/mecabot/cmd_vel', '/cmd_vel'),
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        ign_bridge,
    ])
