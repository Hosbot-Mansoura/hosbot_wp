#!/usr/bin/env python3

# TO INSTALL smbus2
# sudo apt update
# sudo apt install python3-smbus2 i2c-tools -y
# sudo apt install ros-jazzy-robot-localization

import rclpy
from rclpy.node import Node
import smbus2
import time
import math
from sensor_msgs.msg import Imu

class IMUNode(Node):
    def __init__(self):
        node_name = "imu_node"
        super().__init__(node_name)
        ########## [ DECLARE MAIN PARAMETERS ] ##########
        self.declare_parameters(
            namespace='',
            parameters=[
                ('I2C_BUS_CHANNEL',rclpy.Parameter.Type.INTEGER),
                ('IMU_REGISTER_ADDRESS',rclpy.Parameter.Type.INTEGER),
                ('FREQUENCY',rclpy.Parameter.Type.INTEGER),
            ]
        )
        self.i2c_channel = self.get_parameter('I2C_BUS_CHANNEL').value
        self.imu_address = self.get_parameter('IMU_REGISTER_ADDRESS').value
        self.frequency = self.get_parameter('FREQUENCY').value
        self.gyroscope_scale_factor = 131
        self.acceleration_scale_factor = 16384
        self.gravity_value = 9.81
        ########## [ CREATE MAIN OBJECTS ] ##########
        self.bus = smbus2.SMBus(self.i2c_channel)
        ########## [ INITIALIZE IMU SENSOR ] ##########
        self.init()
        ########## [ DECLARE FREQUENCY ] ##########
        timer_period = 1 / self.frequency
        self.create_timer(timer_period, self.read_data)


    def combine(self, msb, lsb):
        val = (msb << 8) | lsb
        if val > 32767:
            val -= 65536
        return val
    
    def convert_deg_to_rad(self,value):
        value *= math.pi / 180  
        return value

    def init(self):
        self.imu_publisher = self.create_publisher(Imu,'/imu/data_raw',10)
        # To reset sensor ( From data sheet ) ==> Chatgpt 
        self.bus.write_byte_data(self.imu_address, 0x7E, 0xB6)
        time.sleep(0.1)
        # 0x11 Value to enable acceleration data ( from data sheet ) ==> Chatgpt
        self.bus.write_byte_data(self.imu_address,0x7E,0x11)
        time.sleep(0.1)
        # 0x15 Value to enable gyroscope data ( from data sheet ) ==> Chatgpt
        self.bus.write_byte_data(self.imu_address,0x7E,0x15)
        time.sleep(0.1)

        # 0x41 Address to set acceleration allowed range
        # 0x03 Value  to set range (+/- 2g) ( from data sheet ) ==> chatgpt
        self.bus.write_byte_data(self.imu_address, 0x41, 0x03)
        time.sleep(0.1)
        # 0x41 Address to set acceleration allowed range
        # 0x03 Value  to set range (+/- 250 deg/s) ( from data sheet ) ==> chatgpt
        self.bus.write_byte_data(self.imu_address, 0x43, 0x03)

    def read_data(self):
        # Range [0-6] for gyroscope 
        # Range [7-12] for acceleration
        ########## [ RAW DATA ] ##########
        imu_msg = Imu()
        gyro_data = self.bus.read_i2c_block_data(self.imu_address, 0x0c, 6)
        acc_data = self.bus.read_i2c_block_data(self.imu_address, 0x12, 6)

        ########## [ GYROSCOPE ] ##########
        gx_raw = self.combine(gyro_data[1], gyro_data[0])
        gy_raw = self.combine(gyro_data[3], gyro_data[2])
        gz_raw = self.combine(gyro_data[5], gyro_data[4])

        # convert to rad/s 
        gx = self.convert_deg_to_rad(gx_raw / self.gyroscope_scale_factor)
        gy = self.convert_deg_to_rad(gy_raw / self.gyroscope_scale_factor)
        gz = self.convert_deg_to_rad(gz_raw / self.gyroscope_scale_factor)
        
        ########## [ ACCELERATION ] ##########
        ax_raw = self.combine(acc_data[1], acc_data[0])
        ay_raw = self.combine(acc_data[3], acc_data[2])
        az_raw = self.combine(acc_data[5], acc_data[4])

        ax_in_g = ax_raw / self.acceleration_scale_factor
        ay_in_g = ay_raw / self.acceleration_scale_factor
        az_in_g = az_raw / self.acceleration_scale_factor

        # convert to m/s²
        ax = ax_in_g * self.gravity_value
        ay = ay_in_g * self.gravity_value
        az = az_in_g * self.gravity_value

        ########## [ CREATE IMU MESSAGE ] ##########
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        imu_msg.orientation_covariance[0] = -1 
        imu_msg.angular_velocity_covariance = [0.02,0,0,0,0.02,0,0,0,0.02]
        imu_msg.linear_acceleration_covariance = [0.1,0,0,0,0.1,0,0,0,0.1]

        ########## [ PUBLISH IMU MESSAGE TO robot_localization ] ##########
        self.imu_publisher.publish(imu_msg)




def main(arg=None):
    rclpy.init(args=arg)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()