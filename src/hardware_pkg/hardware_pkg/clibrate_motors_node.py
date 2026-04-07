#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class CalibrateMotorsNode(Node):
    def __init__(self):
        node_name = "calibrate_motors_node"
        super().__init__(node_name)


def main(arg=None):
    rclpy.init(args=arg)
    node = CalibrateMotorsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()