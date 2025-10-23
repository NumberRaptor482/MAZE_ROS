#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('move_robot_node')

        # --- KEY CHANGE FROM laserfollower.py ---
        # This QoS profile matches the one in your working script.
        # It uses a history depth of 10 and the default 'RELIABLE' reliability,
        # which is what your robot's controller is expecting.
        qos_profile = QoSProfile(depth=10)

        # Create the publisher with the correct topic name and QoS profile
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', qos_profile)

        # Set timers for movement
        self.move_timer_period = 5.0  # seconds (triggers a new movement every 5s)
        self.movement_duration = 2.0  # seconds (the robot moves for this long)

        # Main timer to initiate movement cycles
        self.move_timer = self.create_timer(self.move_timer_period, self.move_cycle_callback)
        self.stop_timer = None
        self.forward = True

        self.get_logger().info('Move robot node started. The robot will move for {}s every {}s.'.format(
            self.movement_duration, self.move_timer_period))

    def move_cycle_callback(self):
        """Initiates a movement and sets a timer to stop it."""
        twist = Twist()
        if self.forward:
            self.get_logger().info('Moving forward...')
            twist.linear.x = 0.2  # Move forward at 0.2 m/s
        else:
            self.get_logger().info('Moving backward...')
            twist.linear.x = -0.2  # Move backward at 0.2 m/s
        
        self.publisher_.publish(twist)
        
        # Toggle direction for the next cycle
        self.forward = not self.forward
        
        # If a stop timer is already running, cancel it
        if self.stop_timer:
            self.stop_timer.cancel()

        # Create a new one-shot timer to stop the robot after the specified duration
        self.stop_timer = self.create_timer(self.movement_duration, self.stop_timer_callback)

    def stop_timer_callback(self):
        """Callback to stop the robot and cancel the one-shot timer."""
        self.get_logger().info('Stopping movement.')
        self.stop_robot()
        self.stop_timer.cancel() # Cancel the timer so it only runs once

    def stop_robot(self):
        """Publishes a zero-velocity message to stop the robot."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    move_robot_node = MoveRobotNode()
    try:
        rclpy.spin(move_robot_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure the robot is stopped before shutting down
        move_robot_node.stop_robot()
        move_robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
