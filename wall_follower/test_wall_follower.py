#!/usr/bin/env python3
import numpy as np
import rclpy
import tf2_ros
import tf_transformations

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from wall_follower.np_encrypt import encode, decode
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



class WallTest(Node):

    
    def __init__(self):
        super().__init__("test_wall_follower")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("pose_topic", "/pose")

        self.declare_parameter("side", 1)
        self.declare_parameter("velocity", 1.)
        self.declare_parameter("desired_distance", 1)
        self.declare_parameter("start_x", -4.)
        self.declare_parameter("start_y", -5.4)
        self.declare_parameter("start_z", 0)
        self.declare_parameter("end_x", 5.)
        self.declare_parameter("end_y", -5.)
        self.declare_parameter("name", "default")


        # Fetch constants from the ROS parameter server
        self.TEST_NAME = self.get_parameter('name').get_parameter_value().string_value
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.POSE_TOPIC = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        self.START_x = self.get_parameter('start_x').get_parameter_value().double_value
        self.START_y = self.get_parameter('start_y').get_parameter_value().double_value
        self.START_z = self.get_parameter('start_z').get_parameter_value().double_value
        self.END_x = self.get_parameter('end_x').get_parameter_value().double_value
        self.END_y = self.get_parameter('end_y').get_parameter_value().double_value
        self.NAME = self.get_parameter('name').get_parameter_value().string_value

        self.get_logger().info('Test Name %s' % (self.TEST_NAME))       


        self.max_time_per_test = 120
        self.end_threshold = 1.0

        self.positions = []
        self.dist_to_end = np.infty
        self.saves = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.start_time = self.get_clock().now()

        # A publisher for navigation commands
        self.pose_pub = self.create_publisher(Pose, "pose" , 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 1)

        # A subscriber to laser scans
        self.create_subscription(LaserScan, self.SCAN_TOPIC, self.laser_callback, 1)

        self.START_POSE = [self.START_x, self.START_y, self.START_z]
        self.END_POSE = [self.END_x, self.END_y]
        self.place_car(self.START_POSE)

        self.moved = False
        self.buffer_count = 0
    
    def place_car(self, pose):
        p = Pose()

        p.position.x = pose[0]
        p.position.y = pose[1]

        # Convert theta to a quaternion
        quaternion = tf_transformations.quaternion_from_euler(0, 0, pose[2])
        p.orientation.y = quaternion[1]
        p.orientation.z = quaternion[2]
        p.orientation.w = quaternion[3]

        self.pose_pub.publish(p)
        self.get_logger().info('Placed Car: %f' % (p.position.x))
        self.get_logger().info('Placed Car: %f' % (p.position.y))


    def laser_callback(self, laser_scan):        

        if self.buffer_count < 30:
            self.place_car(self.START_POSE)
            self.buffer_count += 1
            return

        from_frame_rel = 'base_link'
        to_frame_rel = 'map'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        if not self.moved: 
            diff = np.linalg.norm(np.array([self.START_x, self.START_y]) - np.array([t.transform.translation.x, t.transform.translation.y]))
            if 0.3 < (diff):
                self.place_car(self.START_POSE)
                self.get_logger().info(
                    f'Not at start {self.START_x-t.transform.translation.x}, {self.START_y-t.transform.translation.y}, diff {diff}')
                return
            else:
                self.moved = True
                self.get_logger().info('Moved: %s' % (self.moved))
                self.start_time = self.get_clock().now()
        

        ranges = np.array(laser_scan.ranges, dtype='float32')

        angles = np.linspace(
                laser_scan.angle_min,
                laser_scan.angle_max,
                num = ranges.shape[0])

        # Convert the ranges to Cartesian coordinates.
        # Consider the robot to be facing in the
        # positive x direction.
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Filter out values that are out of range
        # and values on the wrong side
        valid_points = self.SIDE * y > 0
        valid_points = np.logical_and(
                valid_points,
                x < 1.5)
        valid_points = np.logical_and(
                valid_points,
                x > 0.)

        # Compute the average distance
        dists = np.abs(y[valid_points])
        dist = np.sum(dists)/dists.shape[0]
        self.get_logger().info('Avg dist: %f' % (dist))

        
        pos = [t.transform.translation.x, t.transform.translation.y]
        
        time = self.get_clock().now() - self.start_time
        time_d = time.nanoseconds * 1e-9
        self.positions.append([time_d] + pos + [dist])
        self.dist_to_end = np.linalg.norm(np.array(pos) - np.array(self.END_POSE))
        # self.get_logger().info(
        #             f'Time: {time_d}, Max time: {self.max_time_per_test}')

        
        
        if time_d > self.max_time_per_test:
            self.get_logger().info("Test timed out!")
            # Send a message of zero
            stop = AckermannDriveStamped()
            stop.drive.speed = 0.
            stop.drive.steering_angle = 0.
            self.drive_pub.publish(stop)
            self.saves[self.TEST_NAME] = encode(np.array(self.positions))
            np.savez_compressed(self.TEST_NAME+"_log", **self.saves)
            raise SystemExit
        if self.dist_to_end < self.end_threshold:
            self.get_logger().info("Reached the end of the test!")
            stop = AckermannDriveStamped()
            stop.drive.speed = 0.
            stop.drive.steering_angle = 0.
            self.drive_pub.publish(stop)
            self.saves[self.TEST_NAME] = encode(np.array(self.positions))
            np.savez_compressed(self.TEST_NAME+"_log", **self.saves)
            raise SystemExit

    


def main():
    
    rclpy.init()
    wall_follower_test = WallTest()
    try:
        rclpy.spin(wall_follower_test)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    wall_follower_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
