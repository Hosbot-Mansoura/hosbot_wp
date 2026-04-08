#!/usr/bin/env python3

# TO INSTALL smbus2
# sudo apt update
# sudo apt install python3-smbus2 i2c-tools -y

import rclpy
from rclpy.node import Node
import smbus2
import time
import math

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
        self.bus.write_byte_data(self.imu_address,0x7E,0x11)
        time.sleep(0.1)
        self.bus.write_byte_data(self.imu_address,0x7E,0x15)
        time.sleep(0.1)

    def read_data(self):
        data = self.bus.read_i2c_block_data(self.imu_address, 0x0c, 6)
        self.get_logger().info(str(data))
        
        pass

def main(arg=None):
    rclpy.init(args=arg)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()