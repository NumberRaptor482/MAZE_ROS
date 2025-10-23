import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import time
from tf_transformations import euler_from_quaternion

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

        return (dist > 1, (1.0, 0.0))

    def timer_callback(self):
        msg = Twist()
        # msg.linear.x, msg.angular.z


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