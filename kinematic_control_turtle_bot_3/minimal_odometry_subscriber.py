#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion



class MinimalOdometrySubscriber(Node):

    def __init__(self):
        super().__init__(node_name='minimal_odometry_subscriber')
        self.subscriber_ = self.create_subscription(msg_type=Odometry, topic='/odom', callback=self.odom_callback, qos_profile=10)

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.get_logger().info(f"x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")



def main(args=None):
    rclpy.init(args=args)
    node = MinimalOdometrySubscriber()
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
