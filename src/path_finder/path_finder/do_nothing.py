import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

class DoNothing(Node):
    def __init__(self):
        super().__init__('do_nothing_path_finder')

        self.get_logger().info(f'Starting do_nothing path finder')

        self.tf_buffer = Buffer()

        self.get_logger().info(f'Listening for tracked_pose messages')

        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz
  
    def tick(self):
        try:
            # Change 'base_link' if your robot uses base_footprint, etc.
            tf = self.tf_buffer.lookup_transform(
                'map', 'tracked_pose', rclpy.time.Time()
            )
            p = tf.transform.translation
            self.get_logger().info(f"position: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")
        except Exception as e:
            self.get_logger().warn(f"TF not available yet: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DoNothing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()