#!/usr/bin/env python3
# To install main library for this node
#  sudo apt update
#  sudo apt install python3-gpiozero
# '
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PWMOutputDevice, DigitalOutputDevice

class MotorsNode(Node):
    def __init__(self):
        node_name = "motors_node" 
        super().__init__(node_name)
        #### [ DECLARE MAIN PARAMETERS ] ####
        self.declare_parameter('wheel_base', 0.3) # This in meters
        self.declare_parameter('max_speed' , 1.0)
        self.wheel_base = self.get_parameter('wheel_base').value # Distance between wheels
        self.max_speed = self.get_parameter('max_speed').value
        #### [ SUBSCRIBE TO COMMANDS CHANNEL (cmd_vel) ] ####
        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.cmd_to_speed,10)
        #### [ MOTOR PINS ] ####
        self.Left_DIR_PIN = DigitalOutputDevice(17)
        self.Left_PWM_PIN = PWMOutputDevice(18)
        self.Right_DIR_PIN = DigitalOutputDevice(23)
        self.Right_PWM_PIN = PWMOutputDevice(24)

        self.get_logger().info("Motors has been initialized successfully")


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
        vLeft_norm = vLeft_clipped / self.max_speed
        vRight_norm = vRight_clipped / self.max_speed
        
        ##### [ MOTORS SPEEDS ] #####
        # LEFT MOTOR
        self.Left_DIR_PIN.on() if vLeft_clipped >= 0 else self.Left_DIR_PIN.off()
        self.Left_PWM_PIN.value = vLeft_norm
        # RIGHT MOTOR
        self.Right_DIR_PIN.on() if vRight_clipped >= 0 else self.Right_DIR_PIN.off()
        self.Right_PWM_PIN.value = vRight_norm
    
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