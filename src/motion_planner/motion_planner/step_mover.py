import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from interfaces.msg import Step
from math import radians, degrees, pi
import time

BOX_SIZE = 2.5
ANGLE_CUTOFF = 0.001
POS_CUTOFF   = 0.05
ANGULAR_KP = 2
LINEAR_KP = 1
ANGULAR_ADJUST_KP = 0.2

STOPPED = 0
TURN = 1
STRAIGHT = 2

class SimplePose:
    def __init__(self, x: float, y: float, theta: float):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        return f'({self.x}, {self.y}, {self.theta})'


def yaw_from_quat(q):
    # geometry_msgs/Quaternion -> yaw (rad)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class StepMover(Node):
    def __init__(self):
        super().__init__('step_mover_motion_planner')
        self.get_logger().info('Starting step_mover motion planner')

        # publisher for velocity
        self.vel_publisher = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 1)

        # advanced calibration algorithm
        cmd = Twist()
        cmd.angular.z = 1.0
        self.vel_publisher.publish(cmd)
        time.sleep(20)
        cmd = Twist()
        cmd.angular.z = 0.0
        self.vel_publisher.publish(cmd)


        # subscribe to step commands
        self.steps_sub = self.create_subscription(Step, '/step', self.step_command_received, 10)

        # subscribe to position data
        self.pose_sub = self.create_subscription(PoseStamped, '/tracked_pose', self.tracked_pose, 10)

        # timer for motion controller updates
        self.timer = self.create_timer(0.05, self.update_motion)  # seconds

        self.motion_queue = [Step(xstep=0, ystep=1), Step(xstep=1, ystep=0)]
        self.current_pose = None
        self.target_pose = None

        self.state = True

    def stop(self):
        self.vel_publisher.publish(Twist())

    def step_command_received(self, step: Step):
        self.get_logger().info(f"Received step: xstep={step.xstep}, ystep={step.ystep}")
        if abs(self.xstep) > 0 and abs(self.ystep) > 0:
            self.get_logger().info("Invalid step. Must be either x direction or y direction, not both")
        else:
            self.motion_queue.append(step)

    def tracked_pose(self, pose: PoseStamped):
        x = pose.pose.position.x + BOX_SIZE / 2
        y = pose.pose.position.y + BOX_SIZE / 2
        theta = yaw_from_quat(pose.pose.orientation)
        self.current_pose = SimplePose(x, y, theta)

    def round_pose(self, raw_pose):
        x = int(raw_pose.x / BOX_SIZE) * BOX_SIZE + BOX_SIZE / 2
        y = int(raw_pose.y / BOX_SIZE) * BOX_SIZE + BOX_SIZE / 2
        theta = radians(int((degrees(raw_pose.theta) + 45.0) / 90.0) * 90.0)
        return SimplePose(x, y, theta)


    def update_motion(self):
        if self.current_pose is None: return
        if self.target_pose is None:
            if len(self.motion_queue) == 0: return
            self.state = TURN
            next_op = self.motion_queue.pop()
            target_angle = 0
            if next_op.ystep > 0:
                target_angle = pi / 2
            elif next_op.xstep < 0:
                target_angle = pi
            elif next_op.ystep < 0:
                target_angle = 3 * pi / 2
            self.target_pose = self.round_pose(SimplePose(self.current_pose.x + next_op.xstep * BOX_SIZE, self.current_pose.y + next_op.ystep * BOX_SIZE, target_angle))
            print(self.target_pose)

        cmd = Twist()

        if self.state == STOPPED:
            return
        
        if self.state == TURN and abs(self.target_pose.theta - self.current_pose.theta) > ANGLE_CUTOFF:
            cmd.angular.z = -(self.target_pose.theta - self.current_pose.theta) * ANGULAR_KP
        elif self.state == TURN:
            self.state = STRAIGHT
        
        
        if self.state == STRAIGHT and abs(self.target_pose.x - self.current_pose.x) > POS_CUTOFF and (self.target_pose.theta == 0 or self.target_pose.theta == 2) \
                                    or abs(self.target_pose.y - self.current_pose.y) > POS_CUTOFF and (self.target_pose.theta == 1 or self.target_pose.theta == 3):
            if self.target_pose.theta == 1 or self.target_pose.theta == 3:
                cmd.linear.x = max(0.1, abs(self.target_pose.y - self.current_pose.y) * LINEAR_KP)
                print(cmd.linear.x)

                cmd.angular.z = (self.target_pose.x - self.current_pose.x) * ANGULAR_ADJUST_KP

                if self.target_pose.theta == 3:
                    cmd.angular.z = -cmd.angular.z

                if abs(cmd.angular.z) > 0.1:
                    cmd.linear.x = 0.0
                else:
                    cmd.angular.z = 0.0
                print('angular:', cmd.angular.z)
            else:
                cmd.linear.x = max(0.1, abs(self.target_pose.x - self.current_pose.x) * LINEAR_KP)

                cmd.angular.z = (self.target_pose.y - self.current_pose.y) * ANGULAR_ADJUST_KP

                print(self.target_pose.y, self.current_pose.y)

                if self.target_pose.theta == 0:
                    cmd.angular.z = -cmd.angular.z

                if abs(cmd.angular.z) > 0.1:
                    cmd.linear.x = 0.0
                else:
                    cmd.angular.z = 0.0
        elif not self.state == TURN:
            print('stopped')
            if self.state == STOPPED:
                return
            self.state = STOPPED
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            self.target_pose = None

        print('x, theta:', cmd.linear.x, cmd.angular.z)


        self.vel_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = StepMover()
    rclpy.spin(node)
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()