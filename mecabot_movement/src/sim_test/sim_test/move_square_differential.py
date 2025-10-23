#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
import math # Needed for pi

class MoveSquareDifferentialNode(Node):
    def __init__(self):
        super().__init__('move_square_differential_node')

        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', qos_profile)

        # --- Movement Configuration ---
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s (adjust as needed for turn speed)

        # Calculate durations based on speed
        # Time to move forward for one side (e.g., 1 meter)
        self.forward_duration = 1.0 / self.linear_speed # 5 seconds for a 1-meter side
        # Time to turn 90 degrees (pi/2 radians)
        self.turn_duration = (math.pi / 2) / self.angular_speed

        # State machine: 0 for moving forward, 1 for turning
        self.state = 0
        self.cycle_count = 0 # To track completion of the square

        # Create a timer that triggers the next action
        self.timer = self.create_timer(0.5, self.execute_move_state) # Start quickly
        
        self.get_logger().info('Differential drive square node started.')

    def execute_move_state(self):
        # First, cancel the timer to make it a one-shot action for this state
        self.timer.cancel()
        
        twist = Twist()

        if self.state == 0: # State 0: Move Forward
            self.get_logger().info(f'Side {self.cycle_count + 1}/4: Moving forward...')
            twist.linear.x = self.linear_speed
            next_state_delay = self.forward_duration
            self.state = 1 # Next state will be turning
        
        elif self.state == 1: # State 1: Turn
            self.get_logger().info(f'Turn {self.cycle_count + 1}/4: Turning...')
            twist.angular.z = self.angular_speed
            next_state_delay = self.turn_duration
            self.state = 0 # Next state will be moving forward
            self.cycle_count += 1

        self.publisher_.publish(twist)

        # If we have completed the square, reset the cycle
        if self.cycle_count >= 4:
            self.get_logger().info('Square complete. Restarting...')
            self.cycle_count = 0
            # Add a small pause before restarting
            next_state_delay += 1.0

        # Set a new timer to stop this action and trigger the next one
        self.timer = self.create_timer(next_state_delay, self.transition_to_next_state)

    def transition_to_next_state(self):
        """Stop the current movement and immediately trigger the next state."""
        self.stop_robot()
        # The new state is already set, so just call the executor
        self.execute_move_state()

    def stop_robot(self):
        """Publishes a zero-velocity message to stop all movement."""
        self.get_logger().info('Stopping current action...')
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    move_square_node = MoveSquareDifferentialNode()
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
