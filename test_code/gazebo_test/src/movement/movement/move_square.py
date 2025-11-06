import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import time
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan


class MoveSquare(Node):
    def __init__(self):
        super().__init__('move_square')
        # Publisher for /cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 1)
        # Subscriber for /model/vehicle_blue/odometry
        self.odom_sub_ = self.create_subscription(
            Odometry, '/model/vehicle_blue/odometry', self.odom_callback, 10)
        self.imu_sub_ = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10)

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        self.step = 0
        self.start_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.start_angle = 0
        self.position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.target_distance = 1.0  # 1m per side of square
        self.turn_duration = 1.963  # Time for ~90-degree turn (calibrated)
        self.linear_speed = 0.5  # m/s (matches odometry linear.x)
        self.angular_speed = 0.8  # rad/s (tuned for ~90 deg)
        self.turn_start_time = None
        self.stop_time = 0
        self.target_angle = 90
        self.last_imu_update = 0
        self.rotational_velocity = 0
        self.current_time = 0
        self.rotation_start_time = 0
        self.closest_object = 100000
        self.closest_left = 10000
        self.closest_right = 10000
        self.angle_adjustment = 0
        self.last_adjustment = -1000
        self.is_turning = False
        self.just_followed_path = False
        

    def lidar_callback(self, msg: LaserScan):
        # Example: Print out some key info
        #self.get_logger().info(
        #    f"Received scan with {len(msg.ranges)} ranges. "
        #    f"Angle range: {msg.angle_min:.2f} to {msg.angle_max:.2f} radians."
        #)

        # Example: Get the nearest distance (ignore inf/nan)
        valid_ranges = [r for r in msg.ranges if r > 0.0 and r < float('inf')]
        if valid_ranges:
            min_distance = min(valid_ranges)
            #self.get_logger().info(f"Closest obstacle: {min_distance:.2f} m")
            self.closest_object = valid_ranges[len(valid_ranges)//2]

            self.closest_left = valid_ranges[-1]
            self.closest_right = valid_ranges[0]
            print(self.closest_left, self.closest_right)

            l1 = valid_ranges[-1]
            l2 = valid_ranges[-2]
            if l2 - 0.005 > l1 / math.cos(math.radians(180/50)) or self.closest_right < 0.5:
                self.angle_adjustment = -1 # turn left
            elif l2 + 0.005 < l1 / math.cos(math.radians(180/50)) or self.closest_left < 0.5:
                self.angle_adjustment = 1 # turn right
            else:
                self.angle_adjustment = 0
            
            if self.angle_adjustment == -1:
                print("LEFT")
            elif self.angle_adjustment == 1: print("RIGHT")

    def imu_callback(self, msg):
        sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 10E9
        if self.last_imu_update == 0:
            self.last_imu_update = sim_time
            self.rotational_velocity = msg.angular_velocity.z
            return
        
        avg_rv = (msg.angular_velocity.z + self.rotational_velocity) / 2
        self.rotational_velocity = msg.angular_velocity.z

        theta_diff = avg_rv * (sim_time - self.last_imu_update) * 180 / math.pi
        self.last_imu_update = sim_time

        #self.position['theta'] += theta_diff
        

    def odom_callback(self, msg):
        # Update position from odometry
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]

        _, _, self.position['theta'] = euler_from_quaternion(quat)
        self.position['theta'] = math.degrees(self.position['theta'])

        self.current_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 10E9
        #print(self.position)

    def pause(self):
        return (time.time() - self.stop_time > 5, (0.0, 0.0))

    def angular_diff(self, θ1, θ2):
        no_circle = abs(θ1 - θ2)
        circle = abs((360 - max(θ1, θ2)) + min(θ1, θ2))

        if no_circle < circle:
            return no_circle
        else: return -circle

    def rotate(self):
        if self.rotation_start_time == 0:
            self.rotation_start_time = self.current_time
        theta_diff = self.angular_diff(self.position['theta'] + 180, self.target_angle)
        #print(theta_diff, self.position['theta'] + 180, self.target_angle)
        return (self.current_time - self.rotation_start_time >= 3.14, (0.0, -0.5 if abs(theta_diff) > 20 else -0.1))

    def straight(self):
        dist = math.sqrt((self.position['x'] - self.start_position['x']) ** 2 + (self.position['y'] - self.start_position['y']) ** 2)

        return (dist > 2, (1.0, 0.0))

    def timer_callback(self):
        msg = Twist()
        # msg.linear.x, msg.angular.z

        if self.closest_object < 1.1:
            print("wall found", self.closest_object)
            msg.linear.x = 0.0
            msg.angular.z = -0.5
            self.is_turning = True
            self.publisher_.publish(msg)
            time.sleep(1.5)
            return
        
        if self.closest_left > 2 and not self.just_followed_path:
            print("Found left path")
            msg.linear.x = 0.25
            self.publisher_.publish(msg)
            time.sleep(3)

            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.is_turning = True
            self.publisher_.publish(msg)
            time.sleep(2.8)
            self.just_followed_path = True
            return

        msg.linear.x = 0.25 if not self.is_turning else 0.0
        msg.angular.z = 0.0
        if self.last_adjustment != self.angle_adjustment:
            print("Adjusting")
            self.last_adjustment = self.angle_adjustment
            if self.angle_adjustment == -1:
                msg.angular.z = 0.05
            elif self.angle_adjustment == 1:
                msg.angular.z = -0.05
            else:
                self.is_turning = False
                if self.just_followed_path:
                    self.just_followed_path = False
                    msg.linear.x = 0.35
                    self.publisher_.publish(msg)
                    time.sleep(3.5)
                msg.linear.x = 0.25
            self.publisher_.publish(msg)


        if self.closest_object > 1:
            msg.linear.x = 0.5
            msg.angular.z = 0.0
        else:
            msg.angular.z = 0.5 if self.closest_left > self.closest_right else -0.5
            msg.linear.x = 0.0
            #self.publisher_.publish(msg)
            time.sleep(0.157)


        #self.publisher_.publish(msg)
        return

        if self.step == 0:
            result = self.straight()
        elif self.step == 1:
            result = self.pause()
        elif self.step == 2:
            result = self.rotate()
        elif self.step == 3:
            result = self.pause()
        else: result = (True, (0.0, 0.0))

        if result[0]:
            self.step += 1
            self.stop_time = time.time()
            self.start_angle = self.position['theta']
            if self.step == 3:
                self.target_angle = (self.target_angle + 90) % 360
            self.start_position = {'x': self.position['x'], 'y': self.position['y']}
            if self.step > 3: self.step = 0
        print(f"X: {self.position['x']: .1f}, Y: {self.position['y']: .1f}, θ: {self.position['theta']}")
        #print(self.step)
        
        msg.linear.x = result[1][0]
        msg.angular.z = result[1][1]

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoveSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()