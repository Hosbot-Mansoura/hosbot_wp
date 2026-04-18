#!/usr/bin/env python3

from gpiozero import Button
from signal import pause
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

class ButtonNode(Node):
    def __init__(self):
        node_name = "button_node" # Define node name
        super().__init__(node_name)
        self.button = Button(26) # GPIO pin number for the button
        self.button.when_pressed = self.on_button_pressed # Set callback for button press
        self.twist = Twist()
        self.twist.linear.x = 0.05
        self.can_run = False
        self.pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.create_timer(0.1,self.run_motor)

    def on_button_pressed(self):
        self.can_run = not self.can_run
        self.get_logger().info(f"can_run = {self.can_run}")

        
    def run_motor(self):
        if self.can_run:
            self.pub.publish(self.twist)
        else :
            self.pub.publish(Twist())


    

def main(arg=None):
    rclpy.init(args=arg)
    # To do node logic
    node = ButtonNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()