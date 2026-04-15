#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import random

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        
        self.linear_speed = 0.25
        self.angular_speed = 0.6
        self.backward_speed = -0.2
        self.obstacle_distance = 0.5
        self.explore_duration = 600.0
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.cliff_sub = self.create_subscription(Bool, 'cliff_detected', self.cliff_callback, 10)
        
        self.scan_data = None
        self.cliff_detected = False
        self.state = 'FORWARD'
        self.rotate_direction = 1
        self.backup_counter = 0
        
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.random_timer = self.create_timer(3.0, self.random_behavior)
        self.start_time = self.get_clock().now()
        
        self.get_logger().info('Explorer Ready - Autonomous Mode Active')
    
    def scan_callback(self, msg):
        self.scan_data = msg
    
    def cliff_callback(self, msg):
        if msg.data and not self.cliff_detected:
            self.get_logger().warn('CLIFF! Backing up')
        self.cliff_detected = msg.data
    
    def get_min_distance(self, start_angle, end_angle):
        if self.scan_data is None:
            return float('inf')
        
        ranges = self.scan_data.ranges
        angle_min = self.scan_data.angle_min
        angle_inc = self.scan_data.angle_increment
        min_dist = float('inf')
        
        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_inc
            if start_angle <= angle <= end_angle and 0.1 < r < 10.0:
                min_dist = min(min_dist, r)
        
        return min_dist
    
    def random_behavior(self):
        if random.random() < 0.25:
            self.rotate_direction = random.choice([1, -1])
    
    def control_loop(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.explore_duration:
            self.get_logger().info('Exploration Complete')
            self.stop_robot()
            return
        
        twist = Twist()
        
        if self.cliff_detected:
            twist.linear.x = self.backward_speed
            twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)
            return
        
        if self.scan_data is None:
            twist.linear.x = 0.1
            self.cmd_vel_pub.publish(twist)
            return
        
        front = self.get_min_distance(-0.5, 0.5)
        left = self.get_min_distance(0.5, 1.57)
        right = self.get_min_distance(-1.57, -0.5)
        back = self.get_min_distance(2.6, 3.6)
        
        if front < self.obstacle_distance:
            if back > 0.3:
                self.state = 'BACKUP'
                self.backup_counter += 1
                if self.backup_counter > 10:
                    self.state = 'ROTATE'
                    self.backup_counter = 0
                    self.rotate_direction = 1 if left > right else -1
            else:
                self.state = 'ROTATE'
                self.rotate_direction = 1 if left > right else -1
        else:
            self.state = 'FORWARD'
            self.backup_counter = 0
        
        if self.state == 'FORWARD':
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
        elif self.state == 'BACKUP':
            twist.linear.x = self.backward_speed
            twist.angular.z = 0.0
        elif self.state == 'ROTATE':
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed * self.rotate_direction
        
        self.cmd_vel_pub.publish(twist)
    
    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())
    
    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()