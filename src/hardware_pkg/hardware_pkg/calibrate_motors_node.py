#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class CalibrateMotorsNode(Node):
    def __init__(self):
        node_name = "calibrate_motors_node"
        super().__init__(node_name)
        self.l_encoder_pulse_count = 0
        self.r_encoder_pulse_count = 0
        self.get_logger().info("########## START CALIBRATING MOTORS ##########")
        self.l_encoder_sub = self.create_subscription(Int32 , '/calibration/encoder/lift',self.on_left_encoder_data_received,10)
        self.r_encoder_sub = self.create_subscription(Int32 , '/calibration/encoder/right',self.on_right_encoder_data_received,10)

    def on_left_encoder_data_received(self, data:Int32):
        self.l_encoder_pulse_count = data
        self.l_encoder_sub.destroy()


    def on_right_encoder_data_received(self, data:Int32):
        self.r_encoder_pulse_count = data
        self.r_encoder_sub.destroy()
        self.calibrate_motor()

    def calibrate_motor(self):
        logger = self.get_logger()
        logger.info("Left motor pulse count: "+str(self.l_encoder_pulse_count))
        logger.info("Right motor pulse count: "+str(self.r_encoder_pulse_count))
        left_motor_RPM = ((self.l_encoder_pulse_count / 6) / 5.0) / 60
        right_motor_RPM = ((self.r_encoder_pulse_count / 6) / 5.0) / 60
        logger.info("Left motor RPM: "+str(left_motor_RPM))
        logger.info("Right motor RPM: "+str(right_motor_RPM))
        motors_factor =  (right_motor_RPM /left_motor_RPM) if left_motor_RPM > right_motor_RPM else (left_motor_RPM /right_motor_RPM)
        logger.info("Motors factor: " +str(motors_factor))
        self.get_logger().info("########## CALIBRATING MOTORS FINISHED ##########")


def main(arg=None):
    rclpy.init(args=arg)
    node = CalibrateMotorsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()