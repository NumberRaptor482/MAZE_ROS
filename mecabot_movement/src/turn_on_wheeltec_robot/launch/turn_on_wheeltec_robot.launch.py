import os
from pathlib import Path
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                              SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import launch_ros.actions

def generate_launch_description():
    # Get directories
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # --- Declare Launch Arguments ---
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Set to true for simulation, false for real hardware'
    )
    carto_slam_arg = DeclareLaunchArgument('carto_slam', default_value='false')

    use_sim = LaunchConfiguration('use_sim')
    carto_slam = LaunchConfiguration('carto_slam')

    # --- REAL ROBOT Group (launches base_serial.launch.py) ---
    real_robot_group = GroupAction(
        condition=UnlessCondition(use_sim),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
            )
        ]
    )

    # --- SIMULATION Group (sets use_sim_time) ---
    # Gazebo is no longer launched here. We only set the global parameter.
    simulation_group = GroupAction(
        condition=IfCondition(use_sim),
        actions=[
            # This ensures all nodes launched in this context use the simulated clock
            # provided by Gazebo.
            SetEnvironmentVariable('ROS_USE_SIM_TIME', 'true'),
        ]
    )

    # --- Common Nodes (for BOTH simulation and real) ---
    flagship_type = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description.launch.py')),
        launch_arguments={'flagship_mec_dl_robot': 'true', 'use_sim_time': use_sim}.items(),
    )
    
    robot_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_ekf.launch.py')),
        launch_arguments={'carto_slam': carto_slam, 'use_sim_time': use_sim}.items(),
    )
    
    imu_config = Path(get_package_share_directory('turn_on_wheeltec_robot'), 'config', 'imu.yaml')
    base_to_link = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher', name='base_to_link',
        arguments=['0', '0', '0','0', '0','0','base_footprint','base_link'])
        
    base_to_gyro = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher', name='base_to_gyro',
        arguments=['0', '0', '0','0', '0','0','base_footprint','gyro_link'])
        
    imu_filter_node = launch_ros.actions.Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
        parameters=[imu_config, {'use_sim_time': use_sim}])
        
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim}])

    # --- Assemble the Final Launch Description ---
    ld = LaunchDescription()
    
    ld.add_action(use_sim_arg)
    ld.add_action(carto_slam_arg)
    
    ld.add_action(real_robot_group)
    ld.add_action(simulation_group) # This now only sets the sim time param
    
    ld.add_action(flagship_type)
    ld.add_action(base_to_link)
    ld.add_action(base_to_gyro)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(imu_filter_node)
    ld.add_action(robot_ekf)
    
    return ld
