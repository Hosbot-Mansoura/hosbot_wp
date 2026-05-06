#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import math


class UltrasoundNode(Node):
    def __init__(self):
        node_name = "ultrasound_node"
        super().__init__(node_name)
        ##### [ DECLARE MAIN PARAMETERS ] #####
        self.declare_parameters(
            namespace="",
            parameters= [
                ("PORT", rclpy.Parameter.Type.STRING),
                ("BAUDRATE", rclpy.Parameter.Type.INTEGER),
                ("MIN_RANGE", rclpy.Parameter.Type.DOUBLE),
                ("MAX_RANGE", rclpy.Parameter.Type.DOUBLE),
                ("FOV", rclpy.Parameter.Type.DOUBLE),
            ]
        )
        self.port = self.get_parameter("PORT").value
        self.baudrate = self.get_parameter("BAUDRATE").value
        self.min_range = self.get_parameter("MIN_RANGE").value
        self.max_range = self.get_parameter("MAX_RANGE").value
        self.fov = self.get_parameter("FOV").value
        self.sensors_frames = ["ultrasound_1_link", "ultrasound_2_link", "ultrasound_3_link"]
        self.pubs = [
            self.create_publisher(Range,'/ultrasound/sensor_1',10),
            self.create_publisher(Range,'/ultrasound/sensor_2',10),
            self.create_publisher(Range,'/ultrasound/sensor_3',10),
        ]
        self.debug_data = []
        ##### [ CONNECT TO UART PORT ] #####
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
            self.get_logger().info(f"Opened {self.port} at {self.baudrate}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e

        self.create_timer(0.05,self.read_serial_data)

    def read_serial_data(self):
        start_of_frame = self.ser.read(1)
        if start_of_frame != b'\xff': return
        frame_body = self.ser.read(9)
        if len(frame_body) !=9: return
        full_frame = start_of_frame + frame_body
        checksum = sum(full_frame[0:9]) & 0xFF
        if checksum != full_frame[9] : 
            self.get_logger().info("Bad Frame!!")
            return
        ultrasound_1_mm = (full_frame[1] << 8) | full_frame[2]
        ultrasound_2_mm = (full_frame[3] << 8) | full_frame[4]
        ultrasound_3_mm = (full_frame[5] << 8) | full_frame[6]
        # ultrasound_4_mm = (full_frame[7] << 8) | full_frame[8]

        ultrasound_1_m = ultrasound_1_mm / 1000.0
        ultrasound_2_m = ultrasound_2_mm / 1000.0
        ultrasound_3_m = ultrasound_3_mm / 1000.0
        # ultrasound_4_m = ultrasound_4_mm / 1000.0
        data = [ultrasound_1_m,ultrasound_2_m,ultrasound_3_m]
        self.publish_data(data)

    
    def create_range_msg(self , distance , frame_id):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.fov
        # msg.field_of_view = 0.35
        msg.min_range = self.min_range
        # msg.min_range = 0.25
        msg.max_range = self.max_range
        # msg.max_range = 2.0
        msg.range = float(distance)
        return msg



    def publish_data(self , data):
        # self.get_logger().info(str(data))
        for i, distance in enumerate(data):
            # if distance < 0.25 or distance > 4.50: continue
            if distance <= 0 : distance = self.max_range
            elif distance < self.min_range: distance = self.min_range
            elif distance > self.max_range : distance = self.max_range
            msg = self.create_range_msg(distance, self.sensors_frames[i])
            self.pubs[i].publish(msg)



def main(arg=None):
    rclpy.init(args=arg)
    node = UltrasoundNode()
    try:    
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()