#!/usr/bin/env python3
import math
import statistics

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')

        # إعدادات مناسبة للـ mapping لكن بدون بطء أو تقطيع
        self.max_linear_speed = 0.20
        self.max_angular_speed = 0.30
        self.backward_speed = -0.10

        self.safe_distance = 0.65
        self.stop_distance = 0.35
        self.explore_duration = 600.0

        # كلما زادت alpha كانت الاستجابة أسرع
        self.alpha = 0.45

        self.scan_data = None
        self.cliff_detected = False
        self.start_time = self.get_clock().now()

        self.current_linear = 0.0
        self.current_angular = 0.0

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.cliff_sub = self.create_subscription(Bool, 'cliff_detected', self.cliff_callback, 10)

        # 20 Hz أنعم من 10 Hz
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Explorer Ready - Smooth Mapping Mode')

    def scan_callback(self, msg):
        self.scan_data = msg

    def cliff_callback(self, msg):
        self.cliff_detected = msg.data

    def angle_in_range(self, angle, start, end):
        # يدعم wrap-around
        if start <= end:
            return start <= angle <= end
        return angle >= start or angle <= end

    def get_sector_values(self, start_angle, end_angle):
        if self.scan_data is None:
            return []

        values = []
        angle = self.scan_data.angle_min

        for r in self.scan_data.ranges:
            if self.angle_in_range(angle, start_angle, end_angle):
                if math.isfinite(r) and 0.08 < r < 8.0:
                    values.append(r)
            angle += self.scan_data.angle_increment

        return values

    def get_sector_distance(self, start_angle, end_angle):
        values = self.get_sector_values(start_angle, end_angle)
        if not values:
            return float('inf')

        # median أنعم من min وأقل حساسية للقراءات الشاذة
        return statistics.median(values)

    def smooth(self, current, target):
        return current + self.alpha * (target - current)

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def control_loop(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.explore_duration:
            self.stop_robot()
            return

        target_linear = 0.0
        target_angular = 0.0

        # أمان الحافة
        if self.cliff_detected:
            target_linear = self.backward_speed
            target_angular = 0.20

        elif self.scan_data is None:
            target_linear = 0.0
            target_angular = 0.0

        else:
            front = self.get_sector_distance(-0.30, 0.30)
            left = self.get_sector_distance(0.35, 1.20)
            right = self.get_sector_distance(-1.20, -0.35)
            front_left = self.get_sector_distance(0.10, 0.70)
            front_right = self.get_sector_distance(-0.70, -0.10)

            # فرق المساحة بين الجهتين
            error = front_left - front_right

            # deadband لتقليل الرعشة
            if abs(error) < 0.08:
                steering = 0.0
            else:
                steering = self.clamp(0.7 * error, -self.max_angular_speed, self.max_angular_speed)

            # القرار الأساسي
            if front <= self.stop_distance:
                # turn in place عند الاقتراب الشديد
                target_linear = 0.0
                target_angular = self.max_angular_speed if left > right else -self.max_angular_speed

            elif front < self.safe_distance:
                # ابطأ تدريجيًا قرب العائق
                ratio = (front - self.stop_distance) / (self.safe_distance - self.stop_distance)
                ratio = self.clamp(ratio, 0.0, 1.0)

                target_linear = max(0.06, self.max_linear_speed * ratio)
                target_angular = steering

            else:
                # طريق مفتوح
                target_linear = self.max_linear_speed
                target_angular = steering * 0.7

        self.current_linear = self.smooth(self.current_linear, target_linear)
        self.current_angular = self.smooth(self.current_angular, target_angular)

        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        self.cmd_vel_pub.publish(twist)

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