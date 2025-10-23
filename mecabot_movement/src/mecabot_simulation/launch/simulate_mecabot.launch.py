# File: ~/your_ros2_workspace/src/mecabot_simulation/launch/simulate_mecabot.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- Paths and Configurations ---
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_mecabot_simulation = get_package_share_directory('mecabot_simulation')
    
    # Path to your world file
    world_path = os.path.join(pkg_mecabot_simulation, 'worlds', 'mecabot_world.sdf')
    
    # Path to your robot's URDF file
    xacro_file = os.path.join(pkg_mecabot_simulation, 'urdf', 'mecabot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])

    # --- Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=world_path)

    # --- Gazebo Simulation ---
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world, 'verbose': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # --- Robot State Publisher and Spawner ---
    # This node reads the URDF and publishes the robot's state to /tf
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_config
        }]
    )

    # This node spawns the robot from the /robot_description topic into Gazebo[47][8].
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'mecabot'],
        output='screen'
    )

    # --- Bridge (Not needed for /cmd_vel with the diff_drive plugin) ---
    # The gazebo_ros_diff_drive plugin handles the /cmd_vel topic directly,
    # so a separate ros_ign_bridge is no longer necessary for movement.

    # --- Include the Original Robot Launch File ---
    mecabot_move_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mecabot_move_pkg'),
                'launch',
                'move_robot.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # --- Create the Final Launch Description ---
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'))
    ld.add_action(DeclareLaunchArgument('world', default_value=world_path, description='Full path to world file to load'))
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(mecabot_move_launch)

    return ld
