import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Define package paths
    pkg_turn_on_wheeltec_robot = get_package_share_directory('turn_on_wheeltec_robot')
    pkg_mecabot_move = get_package_share_directory('mecabot_move_pkg')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    # Process the URDF file
    xacro_file = os.path.join(pkg_turn_on_wheeltec_robot, 'description', 'mecabot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # --- 1. Start Ignition Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')
        ),
        launch_arguments={'ign_args': '-r -v 4 empty.sdf'}.items(),
    )

    # --- 2. Spawn the Robot ---
    # The 'create' executable is used to spawn entities in Ignition Gazebo
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-topic', 'robot_description', '-entity', 'mecabot'],
        output='screen'
    )

    # --- 3. Robot State Publisher ---
    # Publishes the robot's state and TF tree from the URDF
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # --- 4. Launch the Movement Controller ---
    # This is your move_robot.launch.py
    move_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mecabot_move, 'launch', 'move_robot.launch.py')
        )
    )

    # Create the final launch description
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        move_robot_launch,
    ])

