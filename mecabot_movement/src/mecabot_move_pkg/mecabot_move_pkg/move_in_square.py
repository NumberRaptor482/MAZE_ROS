#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

class MoveSquareNode(Node):
    def __init__(self):
        super().__init__('move_square_node')

        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', qos_profile)

        # Movement Configuration
        self.speed = 0.2
        self.side_length_duration = 2.0
        self.pause_duration = 1.0
        self.total_cycle_time = self.side_length_duration + self.pause_duration

        # State machine for the four sides of the square
        self.state = 0
        
        # --- KEY CHANGE 1 ---
        # Initialize a variable to hold our one-shot timer.
        self.stop_timer = None

        # Main timer to initiate movement cycles
        self.move_timer = self.create_timer(self.total_cycle_time, self.move_cycle_callback)

        self.get_logger().info('Move square node started. The robot will now move in a square.')

    def move_cycle_callback(self):
        """This callback is triggered by the timer to execute one side of the square."""
        twist = Twist()

        if self.state == 0:
            self.get_logger().info('State 0: Moving forward...')
            twist.linear.x = self.speed
        elif self.state == 1:
            self.get_logger().info('State 1: Strafing right...')
            twist.linear.y = -self.speed # Negative Y is right
        elif self.state == 2:
            self.get_logger().info('State 2: Moving backward...')
            twist.linear.x = -self.speed
        elif self.state == 3:
            self.get_logger().info('State 3: Strafing left...')
            twist.linear.y = self.speed # Positive Y is left

        self.publisher_.publish(twist)

        # --- KEY CHANGE 2 ---
        # Create a new one-shot timer to stop the robot and store it in self.stop_timer.
        # This overwrites the previous timer object.
        self.stop_timer = self.create_timer(self.side_length_duration, self.stop_robot_callback)

        # Advance to the next state
        self.state = (self.state + 1) % 4

    def stop_robot_callback(self):
        """Stops the robot's movement and cancels the one-shot timer."""
        self.get_logger().info('Pausing...')
        self.stop_robot()
        
        # --- KEY CHANGE 3 ---
        # Cancel the timer that just triggered this callback. This prevents it from
        # firing again and fixes the AttributeError.
        if self.stop_timer:
            self.stop_timer.cancel()

    def stop_robot(self):
        """Publishes a zero-velocity message to stop all movement."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    move_square_node = MoveSquareNode()
    try:
        rclpy.spin(move_square_node)
    except KeyboardInterrupt:
        pass
    finally:
        move_square_node.stop_robot()
        move_square_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
