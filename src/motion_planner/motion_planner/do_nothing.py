import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from interfaces.msg import Step

class DoNothing(Node):
    def __init__(self):
        super().__init__('do_nothing_motion_planner')

        self.get_logger().info(f'Starting do_nothing motion planner')

        # publisher for velocity
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.get_logger().info('Publishing to /cmd_vel')

        # make it stop and do nothing
        vel = Twist()

        vel.linear.x = 0.0
        vel.angular.z = 0.0

        self.vel_publisher.publish(vel)

        # subscribe to step commands
        self.steps = self.create_subscription(Step, '/step', self.step_command_received, 10)

    def step_command_received(self, step):
        self.get_logger().info(f'Received step: {step.xstep}, {step.ystep}')


def main(args=None):
    rclpy.init(args=args)
    node = DoNothing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()