#!/usr/bin/env python3
from ast import Tuple
from math import pi
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Header, Float64MultiArray
from safe_drive_msgs.msg import SafeDriveMsg

from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS!
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("side", 0)
        self.declare_parameter("velocity", 0.0)
        self.declare_parameter("desired_distance", 0.0)
        self.declare_parameter("safety_topic", "/safety_topic")

        # Fetch constants from the ROS parameter server
        # DO NOT MODIFY THIS! This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        self.SAFETY_TOPIC = self.get_parameter("safety_topic").get_parameter_value().string_value


        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # DO NOT MODIFY THIS!
        self.most_recent_time = 0
        self.most_recent_theta = 0
        self.add_on_set_parameters_callback(self.parameters_callback)

        # TODO: Initialize your publishers and subscribers here
        self.drive_publisher = self.create_publisher(SafeDriveMsg, self.SAFETY_TOPIC, 10)
        self.lidar_subscription = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.laser_scan_callback, 10)
        self.lidar_subscription
        self.line_pub = self.create_publisher(Marker, "/wall", 1)
        self.drive_msg = None

    # TODO: Write your callback functions here
    def send_drive_command(self, steering_angle, line_msg):
        """Sends a drive command based on the input steering angle. No
        other params are changed."""
        safe_drive_msg = SafeDriveMsg()
        self.drive_msg = AckermannDriveStamped()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "racecar"
        self.drive_msg.header = header

        drive = AckermannDrive()
        drive.steering_angle = steering_angle

        drive.speed = self.VELOCITY
        self.drive_msg.drive = drive
        safe_drive_msg.drive_msg = self.drive_msg
        safe_drive_msg.line = line_msg

        self.drive_publisher.publish(safe_drive_msg)


    def laser_scan_callback(self, msg):
        """Laser scan callback."""

        kp_gains = 1.5
        kd_gains = 0.5

        # Linear regression
        line = self.plot_line(msg)
        slope = line[0]
        y_int = line[1]

        # Visualize
        x = np.linspace(-2., 2., num=20)
        y = line[0]*x +line[1]
        VisualizationTools.plot_line(x, y, self.line_pub, frame="/laser")

            # Googled this error function
        dist_to_wall = abs(y_int)/np.sqrt(slope**2 + 1)

        if self.SIDE == 1:
                theta_command = kp_gains*(abs(dist_to_wall) - self.DESIRED_DISTANCE) + kd_gains*(slope)
        else:
                theta_command = -kp_gains*(abs(dist_to_wall) - self.DESIRED_DISTANCE) + kd_gains*(slope)

        line_msg = Float64MultiArray()
        line_msg.data = [slope, y_int]
        # Send drive
        self.send_drive_command(theta_command, line_msg)
        self.most_recent_time = msg.header.stamp.nanosec


    def plot_line(self, scan):
        """Linear regression. Range view max = 9."""

        angle_min = scan.angle_min
        ranges = scan.ranges
        increment = scan.angle_increment
        num_samples = len(ranges)

        edited_ranges = [ranges[k] if ranges[k] < 9 else None for k in range(num_samples)]
        if self.SIDE == -1:
            rng = range(num_samples // 6, 4* num_samples // 6)
        else:
            rng = range(num_samples//2,  5*num_samples//6)

        x = []
        y = []
        for k in rng:
            if edited_ranges[k] is not None:
                x.append(edited_ranges[k]*np.cos(angle_min + increment*k))
                y.append(edited_ranges[k]*np.sin(angle_min + increment*k))

        return np.polyfit(x, y, 1)

    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!

        This is used by the test cases to modify the parameters during testing.
        It's called whenever a parameter is set via 'ros2 param set'.
        """
        for param in params:
            if param.name == 'side':
                self.SIDE = param.value
                self.get_logger().info(f"Updated side to {self.SIDE}")
            elif param.name == 'velocity':
                self.VELOCITY = param.value
                self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
            elif param.name == 'desired_distance':
                self.DESIRED_DISTANCE = param.value
                self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
