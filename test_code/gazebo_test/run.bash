source /home/jeremy9/maze/gazebo_test/install/setup.bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge /lidar2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar2:=/laser_scan
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry &
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist &
ros2 run ros_gz_bridge parameter_bridge /imu@sensor_msgs/msg/Imu@ignition.msgs.IMU &
ros2 run movement move_square &
ign gazebo -v4 -r visualize_lidar.sdf

# for newer ros
gz sim -v 4 -r visualize_lidar.sdf