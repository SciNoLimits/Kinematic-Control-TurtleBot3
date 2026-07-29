#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MinimalVelocityPublisher(Node):
    def __init__(self):
        super().__init__("minimal_velocity_publisher")
        self.publisher_ = self.create_publisher(msg_type=Twist, topic='/cmd_vel', qos_profile=10)
        self.timer_ = self.create_timer(timer_period_sec=0.5, callback=self.send_velocity_cmd_callback)
        self.get_logger().info("Minimal Velocity Publisher has been started")


    def send_velocity_cmd_callback(self):
        msg = Twist()
        msg.linear.x = 0.1      # drive forward at 0.1 m/s
        msg.angular.z = 0.0
        self.publisher_.publish(msg=msg)


def main(args=None):
    rclpy.init(args=args)
    node = MinimalVelocityPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()