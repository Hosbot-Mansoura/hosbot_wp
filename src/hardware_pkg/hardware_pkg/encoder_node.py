#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gpiozero import DigitalInputDevice 
from geometry_msgs.msg import Twist 
from std_msgs.msg import Int32
import time



class Encoder:
    def __init__(self , pin, callback):
        self.sensor = DigitalInputDevice(pin, pull_up=False , bounce_time=0.01)
        self.callback = callback
        self.last_state = 0
        self.sensor.when_activated = self.activated
        self.sensor.when_deactivated = self.deactivated

    def activated(self):
        if self.last_state != 1:
            self.callback() 
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
                ('MODE' , rclpy.Parameter.Type.STRING),
            ]
        )
        self.left_pin = self.get_parameter('LEFT_ENCODER_PIN_NUM').value
        self.right_pin = self.get_parameter('RIGHT_ENCODER_PIN_NUM').value
        self.magnet_count = self.get_parameter('MAGNET_COUNT').value
        self.mode = self.get_parameter('MODE').value
        ##### [ CREATE ENCODER OBJECT ] #####
        self.left_sensor = Encoder(self.left_pin ,self.left_pulse_detected)
        self.right_sensor = Encoder(self.right_pin ,self.right_pulse_detected)
        self.get_logger().info('Encoders has been initialized successfully')
        if self.mode == "Calibration":
            self.calibrate()
        

    def left_pulse_detected(self):
        self.left_pulse_counter += 1
        self.get_logger().info('Left Magnet Detected: '+ str(self.left_pulse_counter))

    def right_pulse_detected(self):
        self.right_pulse_counter += 1
        self.get_logger().info('Right Magnet Detected: '+str(self.right_pulse_counter))


    def calibrate(self):
        twist = Twist()
        twist.linear.x = 0.3
        cmd_pub = self.create_publisher(Twist, '/cmd_vel' ,10)
        left_pub = self.create_publisher(Int32, '/calibration/encoder/left' ,10)
        right_pub = self.create_publisher(Int32, '/calibration/encoder/right' ,10)
        start_time = self.get_clock().now()
        self.get_logger().info('Start time: '+ str(start_time))
        while True:
            now = self.get_clock().now() 
            time_elapsed = ((time- start_time)).nanoseconds / 1e9
            if time_elapsed >= 5 :
                self.get_logger().info('End time: '+str(now))
                break
            cmd_pub.publish(twist)
        left_pub.publish(self.left_pulse_counter)
        right_pub.publish(self.right_pulse_counter)



def main(arg=None):
    rclpy.init(args=arg)
    node = EncoderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()