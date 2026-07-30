#!/usr/bin/env python3
"""Controller for TurtleBot3 using the Siegwart feedback law."""

import math

from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry


import rclpy
from rclpy.node import Node


from tf_transformations import euler_from_quaternion


class SiegwartController(Node):
    """ROS 2 controller node for TurtleBot3 using the Siegwart feedback law."""

    def __init__(self):
        """Initialize the Siegwart Controller node.

        Sets default goal pose, controller gains, hardware limits, tolerances,
        robot state variables, and creates ROS2 subscriber and publisher.
        """

        super().__init__(node_name="tb3_siegwart_controller")

        # Goal Pose Paramater Declaration
        self.declare_parameter("x_goal", 2.0)       # meters
        self.declare_parameter("y_goal", 2.0)       # meters
        self.declare_parameter("theta_goal", 0.0)   # radians
        
        # Goal Pose Get Paramater
        # self.x_goal = self.get_parameter("x_goal").value
        # self.y_goal = self.get_parameter("y_goal").value
        # self.theta_goal = self.get_parameter("theta_goal").value
        
        self.x_goal = self.get_parameter("x_goal").get_parameter_value().double_value
        self.y_goal = self.get_parameter("y_goal").get_parameter_value().double_value
        self.theta_goal = self.get_parameter("theta_goal").get_parameter_value().double_value

        # Controller Gains
        self.k_rho = 0.4
        self.k_alpha = 0.8
        self.k_beta = -0.15

        # Hardware Limits of TurtleBot3
        self.v_max = 0.22  # m/s
        self.w_max = 2.84  # rad/s

        # Goal Tolerance
        self.rho_tol = 0.05  # Stop within 5 cm

        # Robot State - updated by the subscriber  callback
        self.x = 0.0  # meters
        self.y = 0.0  # meters
        self.yaw = 0.0  # radians

        self.subscriber_ = self.create_subscription(
            msg_type=Odometry,
            topic="/odom",
            callback=self.odom_callback,
            qos_profile=10,
        )

        self.publisher_ = self.create_publisher(
            msg_type=Twist, topic="/cmd_vel", qos_profile=10
        )

        self.get_logger().info(
            message=f"Controller Started. Goal: Pose = ({self.x_goal:.3f}, {self.y_goal:.3f}), Orientation = {self.theta_goal:.3f}"
        )
        
        self.control_timer = self.create_timer(0.1,   # 10 Hz
                                               self.control_loop
                                               )
        

    def odom_callback(self, msg: Odometry):
        """
        Called automatically every time /odom publishes.
        Extracts x, y, and yaw from the Odometry message.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def compute_error(self):
        """
        Compute polar error variables (rho , alpha , beta ).
        Identical to the notebook implementation.
        """
        dx = self.x_goal - self.x
        dy = self.y_goal - self.y

        # rho = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        rho = math.hypot(dx, dy)
        phi = math.atan2(dy, dx)
        alpha = phi - self.yaw
        beta = self.theta_goal - phi

        # Normalise to ( -pi , pi]
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        beta = math.atan2(math.sin(beta), math.cos(beta))

        return rho, alpha, beta

    def compute_control(self, rho, alpha, beta):
        """Siegwart feedback control law."""
        v = self.k_rho * rho
        w = self.k_alpha * alpha + self.k_beta * beta

        # Reverse if goal is behind the robot
        if abs(alpha) > math.pi / 2:
            v = -v

        return v, w

    def saturate(self, v, w):
        """Proportional scaling to respect hardware limits"""
        scale = min(1.0, self.v_max / (abs(v) + 1e-9), self.w_max / (abs(w) + 1e-9))
        return scale * v, scale * w

    def stop(self):
        """Publish zero velocity."""
        self.publisher_.publish(Twist())
        self.get_logger().info("Robot Stopped.")

    def control_loop(self):
        """Main control loop"""

        rho, alpha, beta = self.compute_error()

        if rho < self.rho_tol:
            self.get_logger().info(f"Goal reached. rho = {rho:.4f}")
            self.stop()
            self.control_timer.cancel()
            return
        
        v, w = self.compute_control(rho, alpha, beta)
        v, w = self.saturate(v, w)
        
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.publisher_.publish(cmd)
        
        self.get_logger().info(f"rho ={rho:.3f} alpha ={alpha:.3f} beta ={beta:.3f} v ={v:.3f} w ={w:.3f}",
                                throttle_duration_sec=1.0
                                )
            


def main(args=None):
    rclpy.init(args=args)
    node = SiegwartController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
