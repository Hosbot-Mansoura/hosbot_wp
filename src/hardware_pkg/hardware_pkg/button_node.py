#!/usr/bin/env python3

from gpiozero import Button
from signal import pause
import rclpy
from rclpy.node import Node

class ButtonNode(Node):
    def __init__(self):
        node_name = "button_node" # Define node name
        super().__init__(node_name)
        self.button = Button(26) # GPIO pin number for the button
        self.button.when_pressed = self.on_button_pressed # Set callback for button press


    def on_button_pressed(self):
        self.get_logger().info("Button Pressed!") # Log message when button is pressed

    

def main(arg=None):
    rclpy.init(args=arg)
    # To do node logic
    node = ButtonNode()
    rclpy.shutdown()

if __name__ == "__main__":
    main()