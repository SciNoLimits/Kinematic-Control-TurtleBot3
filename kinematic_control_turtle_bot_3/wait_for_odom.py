#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class WaitForOdom(Node):
    def __init__(self):
        super().__init__("wait_for_odom")
        self.get_logger().info('Checking for Odometry...')
        self.subs_ = self.create_subscription(msg_type=Odometry, topic="/odom", callback=self.check_odom_callback, qos_profile=10)
        
    def check_odom_callback(self, msg: Odometry):
        self.get_logger().info('Odometry data received.')
        rclpy.shutdown()
        


def main():
    rclpy.init()
    node = WaitForOdom()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
