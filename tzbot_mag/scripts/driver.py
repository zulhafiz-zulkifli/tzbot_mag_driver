#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

class TzbotMagDriver(Node):
    def __init__(self):
        super().__init__('tzbot_mag_driver')
        self.get_logger().info("tzbot_mag_driver node has started.")
    
def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = TzbotMagDriver()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()