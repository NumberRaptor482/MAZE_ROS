source /home/jeremy9/maze/gazebo_test/install/setup.bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar:=/scan &
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V &
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry &
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist &
ros2 run ros_gz_bridge parameter_bridge /imu@sensor_msgs/msg/Imu@ignition.msgs.IMU &
ros2 run movement move_square &
ign gazebo -v4 -r visualize_lidar.sdf

# for newer ros
gz sim -v 4 -r visualize_lidar.sdf






# Steps for running it with cartographer

# create transform for lidar. this is a temporary solution
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 vehicle_blue/chassis vehicle_blue/lidar_link/gpu_lidar

# start relays
ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar:=/scan &
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V &
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry &
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist &
ros2 run topic_tools relay /model/vehicle_blue/tf /tf &
ros2 run topic_tools relay /model/vehicle_blue/odometry /odom &

# run simulation
gz sim -v 4 -r visualize_lidar.sdf 

# start cartographer
# make sure to install it first with setup.bash
ros2 launch my_cartographer cartographer.launch.py

# to generate map
ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05