import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import time
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

LINEAR_SPEED = 1.0
ANGULAR_SPEED = -0.5236 # 30 degrees per second
time_scale = 1.0

START = 0
MOVE_FORWARD = 1
MOVE_LEFT_FROM_WALL = 2
FINE_ANGLE_ADJUSTMENT_FROM_WALL = 3
MOVE_FORWARD_AFTER_NEW_PATH_FOUND = 4
TURN_LEFT_AFTER_NEW_PATH_FOUND = 5
FINE_ANGLE_AFTER_NEW_PATH_FOUND = 6
MOVE_FORWARD_AFTER_TURN = 7
SLOW_AFTER_FORWARD = 8
VERY_SLOW_AFTER_FORWARD = 9
MOVE_FORWARD_SLOWLY_AFTER_NEW_PATH_FOUND = 10
END = 100

class MoveSquare(Node):
    def __init__(self):
        super().__init__('move_square')
        # Publisher for /cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 1)

        # Subscriber for LIDAR data
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10)

        # Timer for velocity updates
        self.timer = self.create_timer(0.05, self.timer_callback)

        # current state. Used for moving in square, not for current algorithm
        self.state = MOVE_FORWARD

        self.closest_front = 100000
        self.closest_left = 100000
        self.closest_right = 100000

        self.last_msg = None

        self.angle_adjustment = 0  # -1 for left, 1 for right, 0 for none

        self.delta_count = 0
        

    def lidar_callback(self, msg: LaserScan):
        if all(math.isinf(r) for r in msg.ranges):
            self.state = END
            return
        # Example: Get the nearest distance (ignore inf/nan)
        valid_ranges = msg.ranges#[r for r in msg.ranges if r > 0.0 and r < float('inf')]
        if valid_ranges:
            self.closest_front = valid_ranges[len(valid_ranges)//2]
            self.closest_left = valid_ranges[-1]
            self.closest_right = valid_ranges[0]

            if valid_ranges[-1] != float('inf') and valid_ranges[-2] != float('inf'):
                l1 = valid_ranges[-1]
                l2 = valid_ranges[-2]
            else:
                l1 = 1
                l2 = 1
            if l2 - 0.01 > l1 / math.cos(math.radians(180/50)) or self.closest_right < 0.8:
                self.angle_adjustment = -1 # turn left
            elif l2 + 0.01 < l1 / math.cos(math.radians(180/50)) or self.closest_left < 0.8:
                self.angle_adjustment = 1 # turn right
            else:
                self.angle_adjustment = 0

    def execute_state(self):
        print("state:", self.state)
        msg = Twist()
        # msg.linear.x, msg.angular.z
        if self.state == MOVE_FORWARD or self.state == SLOW_AFTER_FORWARD or self.state == VERY_SLOW_AFTER_FORWARD or self.state == MOVE_FORWARD_SLOWLY_AFTER_NEW_PATH_FOUND:
            if self.state == VERY_SLOW_AFTER_FORWARD:
                msg.linear.x = LINEAR_SPEED * 0.3
            elif self.state == SLOW_AFTER_FORWARD or self.state == MOVE_FORWARD_SLOWLY_AFTER_NEW_PATH_FOUND:
                msg.linear.x = LINEAR_SPEED * 0.7
            else:
                msg.linear.x = LINEAR_SPEED
            
            if self.angle_adjustment == -1 or self.closest_right < 0.95:
                if self.closest_right < 0.9:
                    print("TOO CLOSE RIGHT")
                msg.angular.z = -ANGULAR_SPEED / 4 # left
            elif self.angle_adjustment == 1 or self.closest_left < 0.95:
                if self.closest_left < 0.9:
                    print("TOO CLOSE LEFT")
                msg.angular.z = ANGULAR_SPEED / 4 # right
            else:
                msg.angular.z = 0.0
        elif self.state == MOVE_LEFT_FROM_WALL or self.state == TURN_LEFT_AFTER_NEW_PATH_FOUND:
            msg.linear.x = 0.0
            msg.angular.z = ANGULAR_SPEED if self.state == MOVE_LEFT_FROM_WALL else -ANGULAR_SPEED
        elif self.state == FINE_ANGLE_ADJUSTMENT_FROM_WALL or self.state == FINE_ANGLE_AFTER_NEW_PATH_FOUND:
            msg.linear.x = 0.0
            if self.angle_adjustment == -1:
                msg.angular.z = -ANGULAR_SPEED / 4
            elif self.angle_adjustment == 1:
                msg.angular.z = ANGULAR_SPEED / 4
            else:
                msg.angular.z = 0.0
        elif self.state == MOVE_FORWARD_AFTER_NEW_PATH_FOUND or self.state == MOVE_FORWARD_AFTER_TURN:
            msg.linear.x = LINEAR_SPEED
            msg.angular.z = 0.0
        elif self.state == END or self.state == START:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        
        #if self.last_msg == msg:
        #    return
        
        self.last_msg = msg
        self.publisher_.publish(msg)

    def update_state(self):
        if self.state == START:
            self.state = MOVE_FORWARD
        elif self.state == MOVE_FORWARD or self.state == SLOW_AFTER_FORWARD or self.state == VERY_SLOW_AFTER_FORWARD or self.state == MOVE_FORWARD_AFTER_NEW_PATH_FOUND or self.state == MOVE_FORWARD_SLOWLY_AFTER_NEW_PATH_FOUND:
            if self.closest_front < 1.5:
                self.state = SLOW_AFTER_FORWARD
            if self.closest_front < 1.3:
                self.state = VERY_SLOW_AFTER_FORWARD
            if self.closest_front < 1.1:
                self.state = MOVE_LEFT_FROM_WALL
            print(self.state, MOVE_FORWARD_AFTER_NEW_PATH_FOUND)
            if self.closest_left > 2.0 and self.closest_left != 100000 and self.state != MOVE_FORWARD_AFTER_NEW_PATH_FOUND and self.state != MOVE_FORWARD_SLOWLY_AFTER_NEW_PATH_FOUND:
                self.state = MOVE_FORWARD_AFTER_NEW_PATH_FOUND
                self.delta_count = 0

            if self.state == MOVE_FORWARD_AFTER_NEW_PATH_FOUND or self.state == MOVE_FORWARD_SLOWLY_AFTER_NEW_PATH_FOUND:
                print(self.delta_count)
                self.delta_count += 1

                if self.closest_front < 1.2:
                    self.state = TURN_LEFT_AFTER_NEW_PATH_FOUND
                    self.delta_count = 0

                if self.state == MOVE_FORWARD_AFTER_NEW_PATH_FOUND and self.delta_count >= abs(int(0.6 / (LINEAR_SPEED * 0.05 * time_scale))):
                    self.state = MOVE_FORWARD_SLOWLY_AFTER_NEW_PATH_FOUND
                if self.state == MOVE_FORWARD_SLOWLY_AFTER_NEW_PATH_FOUND and self.delta_count >= abs(int(0.8 / (LINEAR_SPEED * 0.05 * time_scale))):
                    self.delta_count = 0
                    self.state = TURN_LEFT_AFTER_NEW_PATH_FOUND
        elif self.state == MOVE_LEFT_FROM_WALL:
            self.delta_count += 1
            if self.delta_count >= abs(int(1.5 / (ANGULAR_SPEED * 0.05 * time_scale))):
                self.delta_count = 0
                self.state = FINE_ANGLE_ADJUSTMENT_FROM_WALL
        elif self.state == FINE_ANGLE_ADJUSTMENT_FROM_WALL:
            if self.angle_adjustment == 0:
                self.state = MOVE_FORWARD
        elif self.state == TURN_LEFT_AFTER_NEW_PATH_FOUND:
            self.delta_count += 1
            if self.delta_count >= abs(int(1.8 / (ANGULAR_SPEED * 0.05 * time_scale))):
                self.delta_count = 0
                self.state = FINE_ANGLE_AFTER_NEW_PATH_FOUND
        elif self.state == FINE_ANGLE_AFTER_NEW_PATH_FOUND:
            if self.angle_adjustment == 0 or self.delta_count >= abs(int(1.0 / (ANGULAR_SPEED / 4 * 0.05 * time_scale))):
                self.state = MOVE_FORWARD_AFTER_TURN
            self.delta_count += 1
        elif self.state == MOVE_FORWARD_AFTER_TURN:
            self.delta_count += 1
            if self.delta_count >= abs(int(1.4 / (LINEAR_SPEED * 0.05 * time_scale))):
                self.delta_count = 0
                self.state = MOVE_FORWARD


    def timer_callback(self):
        self.update_state()
        self.execute_state()


def main(args=None):
    rclpy.init(args=args)
    node = MoveSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
