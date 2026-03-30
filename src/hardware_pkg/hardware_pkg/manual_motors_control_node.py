#!/usr/bin/env python3

#sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy (To install controller pack)


import rclpy
from rclpy.node import Node

class ManualMotorsControlNode(Node):
    def __init__(self):
        node_name = "Manual_motor_control_node"
        super().__init__(node_name)
        


def main(arg=None):
    rclpy.init(args=arg)
    # To do node logic
    node = ManualMotorsControlNode()
    rclpy.shutdown()

if __name__ == "__main__":
    main()