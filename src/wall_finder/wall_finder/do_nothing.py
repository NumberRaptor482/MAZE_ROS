import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from interfaces.msg import Cell, Maze

class DoNothing(Node):
    def __init__(self):
        super().__init__('do_nothing_path_finder')

        self.get_logger().info(f'Starting do_nothing wall finder')

        self.get_logger().info(f'Publishing to the /maze topic')
        self.maze_publisher = self.create_publisher(Maze, '/maze', 1)



def main(args=None):
    rclpy.init(args=args)
    node = DoNothing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()