import rclpy
from rclpy.node import Node

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')
        self.get_logger().info("✅ Logger node has started!")

def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
