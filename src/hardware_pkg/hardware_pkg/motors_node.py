#!/usr/bin/env python3

# ==> To install main library for this node
#  sudo apt update
#  sudo apt install python3-gpiozero
# ==> Fro test 
# sudo apt install ros-jazzy-teleop-twist-keyboard
# ros2 run teleop_twist_keyboard teleop_twist_keyboard

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from std_msgs.msg import Int32


class MotorsNode(Node):
    def __init__(self):
        node_name = "motors_node" 
        super().__init__(node_name)
        #### [ DECLARE MAIN PARAMETERS ] ####
        self.declare_parameters(
            namespace= '',
            parameters=[
            ('LEFT_DIR_PIN_NUM',rclpy.Parameter.Type.INTEGER),
            ('LEFT_PWM_PIN_NUM',rclpy.Parameter.Type.INTEGER),
            ('RIGHT_DIR_PIN_NUM',rclpy.Parameter.Type.INTEGER),
            ('RIGHT_PWM_PIN_NUM',rclpy.Parameter.Type.INTEGER),
            ('LEFT_MOTOR_SCALE',rclpy.Parameter.Type.DOUBLE),
            ('RIGHT_MOTOR_SCALE',rclpy.Parameter.Type.DOUBLE),
            ('WHEEL_BASE',rclpy.Parameter.Type.DOUBLE),
            ('MAX_SPEED',rclpy.Parameter.Type.DOUBLE),
        ])
        self.left_dir_pin = self.get_parameter('LEFT_DIR_PIN_NUM').value
        self.left_pwm_pin = self.get_parameter('LEFT_PWM_PIN_NUM').value
        self.right_dir_pin = self.get_parameter('RIGHT_DIR_PIN_NUM').value
        self.right_pwm_pin = self.get_parameter('RIGHT_PWM_PIN_NUM').value
        self.left_motor_scale = self.get_parameter('LEFT_MOTOR_SCALE').value
        self.right_motor_scale = self.get_parameter('RIGHT_MOTOR_SCALE').value
        self.wheel_base = self.get_parameter('WHEEL_BASE').value
        self.max_speed = self.get_parameter('MAX_SPEED').value
        #### [ MOTOR PINS ] ####
        self.Left_DIR_PIN = DigitalOutputDevice(self.left_dir_pin)
        self.Left_PWM_PIN = PWMOutputDevice(self.left_pwm_pin)
        self.Right_DIR_PIN = DigitalOutputDevice(self.right_dir_pin)
        self.Right_PWM_PIN = PWMOutputDevice(self.right_pwm_pin)
        #### [ SUBSCRIBE TO COMMANDS CHANNEL (cmd_vel) ] ####
        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.cmd_to_speed,10)
        self.publisher_dir_l = self.create_publisher(Int32 , '/motors/left/direction' ,self.publish_dir_left,10)
        self.publisher_dir_t = self.create_publisher(Int32 , '/motors/right/direction' ,self.publish_dir_right,10)


        self.get_logger().info("Motors has been initialized successfully")

    def publish_dir_left(self):
        self.publisher_dir_l.publish(self.Left_DIR_PIN.value)


    def publish_dir_right(self):
        self.publisher_dir_t.publish(self.Right_DIR_PIN.value)

    def cmd_to_speed(self, msg:Twist):
        linear_speed = msg.linear.x
        angle = msg.angular.z
        l_wheel_vel = linear_speed - ((angle * self.wheel_base) / 2)
        r_wheel_vel = linear_speed + ((angle * self.wheel_base) / 2)
        self.set_motor_speed(l_wheel_vel , r_wheel_vel)

    def set_motor_speed(self, vLeft, vRight):
        ##### [ CLIP CMD SPEED ] #####
        vLeft_clipped = max(min(vLeft,self.max_speed) , -self.max_speed)
        vRight_clipped = max(min(vRight,self.max_speed) , -self.max_speed)
       
        ##### [ NORMALIZE TO RANGE [0-1] ] #####
        vLeft_norm = (vLeft_clipped / self.max_speed) * self.left_motor_scale
        vRight_norm = (vRight_clipped / self.max_speed) * self.right_motor_scale
        
        ##### [ MOTORS SPEEDS ] #####
        # LEFT MOTOR
        self.Left_DIR_PIN.on() if vLeft_clipped >= 0 else self.Left_DIR_PIN.off()
        self.Left_PWM_PIN.value = abs(vLeft_norm)
        # RIGHT MOTOR
        self.Right_DIR_PIN.on() if vRight_clipped >= 0 else self.Right_DIR_PIN.off()
        self.Right_PWM_PIN.value = abs(vRight_norm)
    
    def stop(self):
        self.Left_DIR_PIN.off()
        self.Left_PWM_PIN.value = 0
        self.Right_DIR_PIN.off()
        self.Right_PWM_PIN.value = 0



def main(arg=None):
    rclpy.init(args=arg)
    node = MotorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()