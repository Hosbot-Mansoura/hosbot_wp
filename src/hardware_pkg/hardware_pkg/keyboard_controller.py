#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty, select, time

class WASDTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear_speed = 0.2
        self.angular_speed = 0.5

        self.keys = {'w':0, 'a':0, 's':0, 'd':0}
        self.timeout = 0.15

    def get_key(self, timeout=0.02):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            key = sys.stdin.read(1) if rlist else ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def update_keys(self, key):
        if key in self.keys:
            self.keys[key] = time.time()

    def compute(self):
        now = time.time()

        forward  = now - self.keys['w'] < self.timeout
        backward = now - self.keys['s'] < self.timeout
        left     = now - self.keys['a'] < self.timeout
        right    = now - self.keys['d'] < self.timeout

        twist = Twist()

        # Linear
        if forward:
            twist.linear.x += self.linear_speed
        if backward:
            twist.linear.x -= self.linear_speed

        # Angular
        if left:
            twist.angular.z += self.angular_speed
        if right:
            twist.angular.z -= self.angular_speed

        return twist

    def run(self):
        self.get_logger().info("WASD Control | Hold keys | Q to quit")

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)

                key = self.get_key()

                if key:
                    if key == 'q':
                        break
                    self.update_keys(key)

                twist = self.compute()
                self.pub.publish(twist)

        except KeyboardInterrupt:
            pass
        finally:
            self.pub.publish(Twist())
            self.get_logger().info("Stopped")


def main():
    rclpy.init()
    node = WASDTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()