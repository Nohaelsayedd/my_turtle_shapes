#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ShapeNode(Node):
    def __init__(self):
        super().__init__('shape_node')
        self.pub = self.create_publisher(String, 'shape_selection', 10)
        self.get_logger().info('shapeNode started. Type: heart / star / flower / stop / clear / exit')

        # run input loop
        try:
            while rclpy.ok():
                s = input('Enter shape (heart/star/flower/stop/clear/exit): ').strip().lower()
                if not s:
                    continue
                if s == 'exit':
                    self.get_logger().info('Exiting shapeNode.')
                    break
                msg = String()
                msg.data = s
                self.pub.publish(msg)
                self.get_logger().info(f'Published: "{s}"')
        except (KeyboardInterrupt, EOFError):
            pass

def main(args=None):
    rclpy.init(args=args)
    node = ShapeNode()
    # No spin needed after loop, as input is done; just shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()