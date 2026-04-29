#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gpiozero import DigitalInputDevice
from geometry_msgs.msg import Twist 
from std_msgs.msg import Int32
import math
import time
from nav_msgs.msg import Odometry



class Encoder:
    def __init__(self , pin, callback):
        self.sensor = DigitalInputDevice(pin, pull_up=False , bounce_time=0.01)
        self.callback = callback
        self.last_pulse_time = 0.0
        self.min_pulse_interval = 0.003
        self.last_state = 0
        self.sensor.when_activated = self.activated
        self.sensor.when_deactivated = self.deactivated

    def activated(self):
        now = time.monotonic()
        if now - self.last_pulse_time < self.min_pulse_interval:
            return
        if self.last_state != 1:
            self.callback() 
            self.last_pulse_time = now
            
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
                ('WHEEL_L' , rclpy.Parameter.Type.DOUBLE),
                ('WHEEL_RADIUS' , rclpy.Parameter.Type.DOUBLE),
                ('UPDATE_TIME' , rclpy.Parameter.Type.DOUBLE),
            ]
        )
        self.left_pin = self.get_parameter('LEFT_ENCODER_PIN_NUM').value
        self.right_pin = self.get_parameter('RIGHT_ENCODER_PIN_NUM').value
        self.magnet_count = self.get_parameter('MAGNET_COUNT').value
        self.wheel_l = self.get_parameter('WHEEL_L').value
        self.wheel_rad = self.get_parameter('WHEEL_RADIUS').value
        self.update_time = self.get_parameter('UPDATE_TIME').value # pulse rate=v/2πr​×N N must be 2-5
        self.l_dir = None
        self.r_dir = None
        self.last_time = time.monotonic()
        self.x_pose = 0.0
        self.y_pose = 0.0
        self.yaw = 0.0
        ##### [ CREATE ENCODER OBJECT ] #####
        self.left_sensor = Encoder(self.left_pin ,self.left_pulse_detected)
        self.right_sensor = Encoder(self.right_pin ,self.right_pulse_detected)
        self.l_dir_sub = self.create_subscription(Int32,'/motors/left/direction',self.set_left_dir,10)
        self.r_dir_sub = self.create_subscription(Int32,'/motors/right/direction',self.set_right_dir,10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.get_logger().info('Encoders has been initialized successfully')
        self.create_timer(self.update_time ,self.update_odometry)


    def left_pulse_detected(self):
        self.left_pulse_counter += 1

    def right_pulse_detected(self):
        self.right_pulse_counter += 1

    def reset(self):
        self.left_pulse_counter  = 0
        self.right_pulse_counter = 0

    def set_left_dir(self , data:Int32):
        self.l_dir = data.data

    def set_right_dir(self , data:Int32):
        self.r_dir = data.data

    def publish_zero_odom(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        odom_msg.pose.pose.position.x = self.x_pose
        odom_msg.pose.pose.position.y = self.y_pose
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        odom_msg.pose.covariance = [
            0.05, 0,    0,    0,    0,    0,
            0,    0.05, 0,    0,    0,    0,
            0,    0,    9999, 0,    0,    0,
            0,    0,    0,    9999, 0,    0,
            0,    0,    0,    0,    9999, 0,
            0,    0,    0,    0,    0,    0.1
        ]
        odom_msg.twist.covariance = [
            0.05, 0,    0,    0,    0,    0,
            0,    0.05, 0,    0,    0,    0,
            0,    0,    9999, 0,    0,    0,
            0,    0,    0,    9999, 0,    0,
            0,    0,    0,    0,    9999, 0,
            0,    0,    0,    0,    0,    0.1
        ]
        self.odom_pub.publish(odom_msg)

    def update_odometry(self):
        if self.r_dir is None or self.l_dir is None:
            self.reset()
            self.last_time = time.monotonic()   
            return
        ########## [ GET MOTORS DIRECTION ] ##########
        pR = self.right_pulse_counter if self.r_dir == 1 else self.right_pulse_counter * -1
        pL = self.left_pulse_counter  if self.l_dir == 1 else self.left_pulse_counter  * -1

        if pR == 0 and pL == 0:
            self.reset()
            self.last_time = time.monotonic()
            self.publish_zero_odom()
            return

        ########## [ RESET PULSE COUNTERS ] ##########
        self.reset()

        ########## [ GET ROTATE COUNT ] ##########
        rot_l_count = pL / self.magnet_count
        rot_r_count = pR / self.magnet_count
        
        ########## [ GET DELTA ANGULAR ] ##########
        dphiL = (2 * math.pi) *  rot_l_count # delta angular left
        dphiR = (2 * math.pi) *  rot_r_count # delta angular right

        ########## [ GET ANGULAR VELOCITY ] ##########
        current_time = time.monotonic()
        dt = current_time - self.last_time
        self.last_time = current_time
        if dt <=0 : 
            return
        wl = dphiL / dt # angular velocity left
        wr = dphiR / dt # angular velocity right

        ########## [ GET LINEAR VELOCITY ] ##########
        vL = self.wheel_rad * wl # linear velocity left
        vR = self.wheel_rad * wr # linear velocity right

        ########## [ GET ROBOT ODOMETRY ] ##########
        vx = (vR + vL) / 2  # linear velocity for robot
        wz = (vR - vL ) / self.wheel_l # angular velocity for robot 

        ########## [ CALCULATRE ROBOTE POSE ] ##########
        ds = vx * dt
        dyaw = wz * dt

        self.x_pose += ds * math.cos(self.yaw + dyaw / 2.0)
        self.y_pose += ds * math.sin(self.yaw + dyaw / 2.0)
        self.yaw += dyaw
        self.yaw = math.atan2(math.sin(self.yaw) , math.cos(self.yaw))

        vx  = 0.0 if abs(vx) < 1e-3 else vx
        wz  = 0.0 if abs(wz) < 1e-3 else wz

        ########## [ PUBLISH ODOMETRY ] ##########
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x_pose
        odom_msg.pose.pose.position.y = self.y_pose
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = wz
        odom_msg.pose.covariance = [
            0.05, 0,    0,    0,    0,    0,
            0,    0.05, 0,    0,    0,    0,
            0,    0,    9999, 0,    0,    0,
            0,    0,    0,    9999, 0,    0,
            0,    0,    0,    0,    9999, 0,
            0,    0,    0,    0,    0,    0.1
        ]
        odom_msg.twist.covariance = [
            0.05, 0,    0,    0,    0,    0,
            0,    0.05, 0,    0,    0,    0,
            0,    0,    9999, 0,    0,    0,
            0,    0,    0,    9999, 0,    0,
            0,    0,    0,    0,    9999, 0,
            0,    0,    0,    0,    0,    0.1
        ]
        self.odom_pub.publish(odom_msg)



def main(arg=None):
    rclpy.init(args=arg)
    node = EncoderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()