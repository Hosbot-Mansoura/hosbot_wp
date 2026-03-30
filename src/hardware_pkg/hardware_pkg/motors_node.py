#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class MotorsNode(Node):
    def __init__(self):
        node_name = "motors_node" 
        super().__init__(node_name)
        #### [ DECLARE MAIN PARAMETERS ] ####
        self.declare_parameter('wheel_base', 0.3) # This in meters
        self.declare_parameter('max_speed' , 1.0)
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        #### [ SUBSCRIBE TO COMMANDS CHANNEL (cmd_vel) ] ####
        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.motor_cmd,10)
        #### [ MOTOR PINS ] ####
        self.Left_DIR = 17
        self.Right_DIR = 23
        self.Left_PWM = 18
        self.Right_PWM = 24
        #### [ MOTOR PINS SETUP ] ####
        GPIO.setup(self.Left_DIR , GPIO.OUT)
        GPIO.setup(self.Right_DIR , GPIO.OUT)
        GPIO.setup(self.Left_PWM , GPIO.OUT)
        GPIO.setup(self.Right_PWM , GPIO.OUT)
        #### [ INITIAL MOTORS VALUES ] ####
        self.pwm_left = GPIO.PWM(self.Left_PWM,1000)
        self.pwm_right = GPIO.PWM(self.Right_PWM , 1000)
        self.pwm_left.start(0)
        self.pwm_right.start(0)
        self.get_logger().info("Motors has been initialized successfully")



    def motor_cmd(self, msg:Twist):
        linear = msg.linear.x
        angular = msg.angular.z
        v_left = linear - (angular * (self.wheel_base / 2.0))
        v_right = linear + (angular * (self.wheel_base / 2.0))
        self.set_motor_speed(v_left , v_right)
        

    def set_motor_speed(self , left , right):
        ##### [ SET MOTOR DIRECTION ] #####
        left_dir = GPIO.HIGH if left >=0 else GPIO.LOW
        right_dir = GPIO.HIGH if right >=0 else GPIO.LOW
        GPIO.output(self.Left_DIR , left_dir)
        GPIO.output(self.Right_DIR , right_dir)
        ##### [ SET MOTOR SPEED ] #####
        left_speed = self.speed_pwm_converter(left)
        right_speed = self.speed_pwm_converter(right)
        self.pwm_left.ChangeDutyCycle(abs(left_speed))
        self.pwm_right.ChangeDutyCycle(abs(right_speed))
        

    def speed_pwm_converter(self,speed):
        speed = max(min(speed,self.max_speed), -self.max_speed)
        pwm_value = (speed / self.max_speed) * 100.0
        return pwm_value


def main(arg=None):
    rclpy.init(args=arg)
    GPIO.setmode(GPIO.BCM)
    node = MotorsNode()
    rclpy.spin(node)
    node.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == "__main__":
    main()