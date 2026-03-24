from interfaces.msg import Step
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

class DoNothing(Node):
    def __init__(self):
        super().__init__('do_nothing_path_finder')

        self.get_logger().info(f'Starting do_nothing path finder')

        self.step_publisher = self.create_publisher(Step, '/step', 10)

        step = Step()
        step.xstep = 1
        step.ystep = 0
        self.step_publisher.publish(step)


def main(args=None):
    rclpy.init(args=args)
    node = DoNothing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()