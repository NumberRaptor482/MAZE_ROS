import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # --- Paths ---
    pkg_share = get_package_share_directory('sim_test')
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'mecabot.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'mecabot_world.sdf')
    controllers_yaml_path = os.path.join(pkg_share, 'config', 'wheeltec_controllers.yaml')
    
    # --- Robot Description ---
    robot_description_config = xacro.process_file(urdf_file_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # --- Gazebo (Ignition/Fortress) ---
    launch_gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '--render-engine', 'ogre', '-r', world_path],
        output='screen'
    )

    # --- Robot State Publisher ---
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # --- Spawn Robot into Gazebo ---
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'wheeltec_robot']
    )

    # --- Spawners (for controllers) ---
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        launch_gz_sim,
        spawn_entity,
        spawn_joint_state_broadcaster,
        spawn_diff_drive_controller
    ])
