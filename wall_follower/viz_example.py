#!/usr/bin/env python3

"""
This is an example of how to use the VisualizationTools class to publish a line 
to the /wall topic.

You do not need this for your wall follower!
"""

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from wall_follower.visualization_tools import VisualizationTools  # Assuming VisualizationTools is a class in visualization_tools module

class LinePublisher(Node):

        
        # the topics to publish and subscribe to
    WALL_TOPIC = "/wall"

    def __init__(self):
        super().__init__('line_publisher')

        self.declare_parameter('wall_follower/scan_topic', '/scan')
        self.SCAN_TOPIC = self.get_parameter('wall_follower/scan_topic').get_parameter_value().string_value

        # a publisher for our line marker
        self.line_pub = self.create_publisher(Marker, self.WALL_TOPIC, 1)

        # a subscriber to get the laserscan data
        self.create_subscription(LaserScan, self.SCAN_TOPIC, self.laser_callback, 10)

    def laser_callback(self, laser_scan):
        # x and y should be points on your detected wall
        # here we are just plotting a parabola as a demo
        x = np.linspace(-2., 2., num=20)
        y = np.square(x)
        VisualizationTools.plot_line(x, y, self.line_pub, frame="/laser")


def main(args=None):
    rclpy.init(args=args)
    line_publisher = LinePublisher()
    rclpy.spin(line_publisher)
    line_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()