import rclpy
from rclpy.node import Node

from bearmax_msgs import StackCommand

DEFAULT_WS_URL = 'ws://localhost'

class StackConnector(Node):
    def __init__(self):
        super().__init__('stack_connector')

        self.declare_parameter('ws_url', )

    def timer_callback(self):
        self.get_logger().info('Test Log Entry')

def main(args=None):
    rclpy.init(args=args)

    stack_connector = StackConnector()

    rclpy.spin(stack_connector)

    stack_connector.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
