import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        # Subscribe to /lidar2
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Lidar subscriber started, listening to /lidar2')

    def lidar_callback(self, msg: LaserScan):
        # Example: Print out some key info
        self.get_logger().info(
            f"Received scan with {len(msg.ranges)} ranges. "
            f"Angle range: {msg.angle_min:.2f} to {msg.angle_max:.2f} radians."
        )

        # Example: Get the nearest distance (ignore inf/nan)
        valid_ranges = [r for r in msg.ranges if r > 0.0 and r < float('inf')]
        if valid_ranges:
            min_distance = min(valid_ranges)
            self.get_logger().info(f"Closest obstacle: {min_distance:.2f} m")


def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
