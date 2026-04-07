#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gpiozero import DigitalInputDevice 
from geometry_msgs.msg import Twist

class Encoder:
    def __init__(self, pin, callback):
        self.sensor = DigitalInputDevice(pin, pull_up=False , bounce_time=0.01)
        self.callback = callback
        self.last_state = 0
        self.sensor.when_activated = self.activated
        self.sensor.when_deactivated = self.deactivated

    def activated(self):
        if self.last_state != 1:
            self.callback 
        self.last_state = 1

    def deactivated(self):
        self.last_state = 0

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
        self.left_pin = self.get_parameter('LEFT_ENCODER_PIN_NUM').value
        self.right_pin = self.get_parameter('RIGHT_ENCODER_PIN_NUM').value
        self.magnet_count = self.get_parameter('MAGNET_COUNT').value
        ##### [ CREATE ENCODER OBJECT ] #####
        # self.left_encoder = DigitalInputDevice(self.left_pin, pull_up = True, bounce_time = 0.001)
        # self.right_encoder = DigitalInputDevice(self.right_pin, pull_up = True, bounce_time = 0.001)
        self.left_sensor = Encoder(self.left_pin ,self.left_pulse_detected)
        self.right_sensor = Encoder(self.right_pin ,self.right_pulse_detected)

        self.test_en()
        self.get_logger().info('Encoders has been initialized successfully')

    def left_pulse_detected(self):
        self.get_logger().info('Left Magnet Detected')

    def right_pulse_detected(self):
        self.get_logger().info('Right Magnet Detected')

    def test_en(self):
        twist :Twist = Twist()
        while (self.left_pin <= 6 or self.right_pin <= 6):
            twist.linear.x = 0.3
            publisher = self.create_publisher(Twist ,'/cmd_vel',10)
            publisher.publish(twist)

    def on_left_pulse(self):
        pass

    def on_right_pulse(self):
        pass 

 







def main(arg=None):
    rclpy.init(args=arg)
    node = EncoderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()