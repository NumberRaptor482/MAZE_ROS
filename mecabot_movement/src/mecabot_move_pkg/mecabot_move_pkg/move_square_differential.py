#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from transforms3d.euler import quat2euler

class MoveSquareDifferentialNode(Node):
    def __init__(self):
        super().__init__('move_square_differential_node')
        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile)

        # --- Movement Configuration ---
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.goal_distance = 1.0
        self.goal_angle = math.pi / 2

        # State machine and variables
        self.state = 'moving_forward'
        self.start_position = None
        self.start_angle = None
        self.current_pose = None
        self.cycle_count = 0
        
        # A single, persistent timer that runs the state machine
        self.timer = self.create_timer(0.02, self.execute_state_machine) # 50Hz
        self.get_logger().info('Closed-loop square node started.')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def execute_state_machine(self):
        if self.current_pose is None:
            self.get_logger().warn('Waiting for odometry data...', throttle_duration_sec=2)
            return

        twist = Twist()

        if self.state == 'moving_forward':
            if self.start_position is None:
                self.start_position = self.current_pose.position
            
            dist_moved = math.sqrt(
                (self.current_pose.position.x - self.start_position.x)**2 +
                (self.current_pose.position.y - self.start_position.y)**2)
            
            if dist_moved < self.goal_distance:
                twist.linear.x = self.linear_speed
            else:
                self.stop_robot()
                self.state = 'turning'
                self.start_position = None
                self.get_logger().info(f'Side {self.cycle_count + 1} complete. Turning.')

        elif self.state == 'turning':
            if self.start_angle is None:
                _, _, self.start_angle = quat2euler([
                    self.current_pose.orientation.w, self.current_pose.orientation.x,
                    self.current_pose.orientation.y, self.current_pose.orientation.z])
            
            _, _, current_angle = quat2euler([
                self.current_pose.orientation.w, self.current_pose.orientation.x,
                self.current_pose.orientation.y, self.current_pose.orientation.z])
            
            angle_turned = abs(current_angle - self.start_angle)
            if angle_turned > math.pi:
                angle_turned = 2 * math.pi - angle_turned

            if angle_turned < self.goal_angle:
                twist.angular.z = self.angular_speed
            else:
                self.stop_robot()
                self.cycle_count += 1
                if self.cycle_count >= 4:
                    self.get_logger().info('Square complete! Stopping.')
                    self.timer.cancel()
                    return
                self.state = 'moving_forward'
                self.start_angle = None
                self.get_logger().info('Turn complete. Starting next side.')

        self.publisher_.publish(twist)

    def stop_robot(self):
        twist = Twist()
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
