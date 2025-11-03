ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry &
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist &
ros2 run ros_gz_bridge parameter_bridge /imu@sensor_msgs/msg/Imu@ignition.msgs.IMU &
ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan &
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V &
