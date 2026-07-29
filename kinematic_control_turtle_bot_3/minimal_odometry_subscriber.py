#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class MinimalOdometrySubscriber(Node):

    def __init__(self):
        super().__init__(node_name='minimal_odometry_subscriber')
        self.subscriber_ = self.create_subscription(msg_type=Odometry, topic='/odom', callback=self.odom_callback, qos_profile=10)

    def odom_callback(self, msg: Odometry):
        self.get_logger().info(
            f"x={msg.pose.pose.position.x:.2f}, "
            f"y={msg.pose.pose.position.y:.2f}"
        )



def main(args=None):
    rclpy.init(args=args)
    node = MinimalOdometrySubscriber()
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
