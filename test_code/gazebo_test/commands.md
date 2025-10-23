##  Relay ign odometry to ros2 odometry
```ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry```

## Relay ros2 cmd_vel to ign cmd_vel
```ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist```

## Run movement
```ros2 run movement move_square```

## Run simulation
```ign gazebo -v4 --render-engine ogre -r visualize_lidar.sdf```