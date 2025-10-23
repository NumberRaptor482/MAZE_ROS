source /home/jeremy9/maze/gazebo_test/install/setup.bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry &
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist &
ros2 run movement move_square &
ign gazebo -v4 --render-engine ogre -r visualize_lidar.sdf