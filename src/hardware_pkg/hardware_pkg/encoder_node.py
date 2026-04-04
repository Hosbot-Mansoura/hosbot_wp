#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gpiozero import DigitalInputDevice 


class EncoderNode(Node):
    def __init__(self):
        node_name = "encoder_node"
        super().__init__(node_name)
        ##### [ DECLARE MAIN PARAMETERS ] #####
        self.left_pulse_counter = 0
        self.right_pulse_counter = 0
        self.declare_parameters(
            namespace='',
            parameters=[
                ('LEFT_ENCODER_PIN_NUM' , rclpy.Parameter.Type.INTEGER),
                ('RIGHT_ENCODER_PIN_NUM' , rclpy.Parameter.Type.INTEGER),
                ('MAGNET_COUNT' , rclpy.Parameter.Type.INTEGER),
            ]
        )
        self.left_pin = self.get_parameter('LEFT_ENCODER_PIN_NUM')
        self.right_pin = self.get_parameter('RIGHT_ENCODER_PIN_NUM')
        self.magnet_count = self.get_parameter('MAGNET_COUNT')
        ##### [ CREATE ENCODER OBJECT ] #####
        self.left_encoder = DigitalInputDevice(self.left_pin, pull_up = True, bounce_time = 0.02)
        self.right_encoder = DigitalInputDevice(self.right_pin, pull_up = True, bounce_time = 0.02)
        ##### [ MAGNET DETECTED ] #####
        self.left_encoder.when_activated = lambda: self.on_pulse_detected(self,is_left= True)
        self.right_encoder.when_activated = lambda: self.on_pulse_detected(self,is_left= False)


    def on_pulse_detected(self, is_left = True):
        if is_left :
            self.get_logger().info('Left Pulse Detected')
        else :
            self.get_logger().info('Right Pulse Detected')







def main(arg=None):
    rclpy.init(args=arg)
    node = EncoderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()