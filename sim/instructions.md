## Running simulation in Gazebo

1. Create a transform topic to tell cartographer where everything is on the robot. This is a temporary solution.
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0.5 0 0 0 vehicle_blue/chassis vehicle_blue/lidar_link/gpu_lidar
```

2. In a separate terminal, start bridge from gazebo to ros2 to create LiDAR topic.
```bash
ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar:=/scan &
```

3. Start bridges from gazebo to ros2 for transform data, odometry data, velocity commands. 
```bash
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock &
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V &
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry &
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist &
ros2 run topic_tools relay /model/vehicle_blue/tf /tf &
ros2 run topic_tools relay /model/vehicle_blue/odometry /odom &
```

4. In a separate terminal, start the gazebo simulation
```bash
gz sim -v4 -r sim/main_sim.sdf
```

5. In a separate terminal, start cartographer. Make sure that the project was installed first using ```source install/setup.bash```
```bash
ros2 launch cartographer cartographer.launch.py
```

6. In a separate terminal, use this to generate a global map. By default cartographer only outputs sections of the map instead of the 
entire map. This is optional. 
```bash
ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05
```

7. Start visualization in a separate terminal.
```bash
rviz2
```