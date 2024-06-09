#!/usr/bin/env python3

import gtsam.noiseModel
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import gtsam
from gtsam import symbol_shorthand as sh
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import tf_transformations
import numpy as np
import utm
from statistics import mean 

# ROS2 node estimating robot position from Odometry, Landmark and GPS
# Jakub Junkiert 2024

class SimpleOdometryNode(Node):
    def __init__(self):
        super().__init__('simple_odometry_node')

        # Subcribers for odom, GPS and IMU
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, 'gps/data', self.gps_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)

        # Estimated pose publisher
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'estimated_pose', 10)
        
        # Graph and optimizer init
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()
        self.optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimate)
        
        self.prev_pose = None
        self.key = 0

        self.first = True


    # Odometry callback
    def odom_callback(self, msg):
        # self.get_logger().info('Received odom position and rotation')

        # Read odom position and orientation and convert it to gtsam.Pose3 type
        current_pose = gtsam.Pose3(gtsam.Rot3.Quaternion(msg.pose.pose.orientation.w,
                                                         msg.pose.pose.orientation.x,
                                                         msg.pose.pose.orientation.y,
                                                         msg.pose.pose.orientation.z),
                                   gtsam.Point3(msg.pose.pose.position.x,
                                                msg.pose.pose.position.y,
                                                msg.pose.pose.position.z))
        
        # Check if it is the beggining of graph
        if self.prev_pose is None:
            # Add first node woth prior factor
            self.graph.add(gtsam.PriorFactorPose3(sh.O(self.key), current_pose, gtsam.noiseModel.Isotropic.Sigma(6, 0.01)))
            self.initial_estimate.insert(sh.O(self.key), current_pose)
        else:
            self.key += 1

            # Set up odom noise
            odom_noise = gtsam.noiseModel.Isotropic.Sigma(6, 0.01)

            # Add new node and between factor
            odometry_factor = gtsam.BetweenFactorPose3(sh.O(self.key - 1), sh.O(self.key), self.prev_pose.between(current_pose), odom_noise)
            self.graph.add(odometry_factor)
            self.initial_estimate.insert(sh.O(self.key), current_pose)

        self.prev_pose = current_pose

        # Execute graph optimization
        self.optimize()


    # GPS callback
    def gps_callback(self, msg):
        # Convert from LatLon to metric coordinates
        utm_loc = utm.from_latlon(msg.latitude, msg.longitude)

        # Subtract in order to have 0,0 in the center of the world
        gps_x = utm_loc[0] - 631412.213
        gps_y = utm_loc[1] - 5808345.693
        self.get_logger().info('Received GPS pos: {0:.4f}, {1:.4f}'.format(gps_x, gps_y))
        
        # Set up GPS noise value
        gps_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.2)

        # Add GPS position to graph, place it next to last odom node
        self.graph.add(gtsam.GPSFactor(sh.O(self.key), [-gps_x, -gps_y, 0.], gps_noise))


    # Laser callback
    def laser_callback(self, msg):
        
        # Localise landmark
        prev_state = False
        middle = False
        angle_min = 0
        angle_max = 0
        distances = []

        for angle, dis in enumerate(msg.ranges):
            if(dis < 10):
                if angle == 0 and dis < 10:
                    middle = True
                if not prev_state:
                    prev_state = True
                    angle_min = angle
                distances.append(dis)
            if(dis > 10 and prev_state):
                prev_state = False
                angle_max = angle

        # Check if landmark was detected
        if angle_min != 0 and angle_max != 0:
            if not middle:
                center_angle = (angle_max + angle_min) / 2.0
            else:
                center_angle = (angle_max - 360 + angle_min) / 2.0
            meas_distance = mean(distances) + 0.0675

            if self.first:
                self.graph.push_back(gtsam.PriorFactorPoint3(sh.L(0), gtsam.Point3(2.5, 1.5, 0.0), gtsam.noiseModel.Isotropic.Sigma(3, 0.5)))
                self.initial_estimate.insert(sh.L(0), gtsam.Point3(2.5, 1.5, 0.0))
                self.first = False
            
            # Convert from degree to x,y on unit circle
            x = np.cos(center_angle * np.pi/180.)
            y = np.sin(center_angle * np.pi/180.)

            # Add to graph
            
            landmark_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.5)
            print(landmark_noise)
            self.graph.add(gtsam.BearingRangeFactor3D(sh.O(self.key), sh.L(0), gtsam.Unit3([x, y, 0.]), meas_distance, landmark_noise))

            self.get_logger().info('Landmark localised: {0:.1f} deg, {1:.2f} m'.format(center_angle, meas_distance))
        else:
            self.get_logger().warn('Landmark not found, current position is probably too far from its location')


    # Function to optimize graph and, in result, optain current estimated pose
    def optimize(self):
        # Update optimizer
        self.optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimate)

        # Optimize only if at least two nodes exists
        if self.key > 1:
            result = self.optimizer.optimize()

            # Get pose and pose error
            optimized_pose = result.atPose3(sh.O(self.key-1))
            marginals = gtsam.Marginals(self.graph, result)

            # Execute function publishing pose
            self.publish_pose(optimized_pose, marginals)


    # Function to publish estimated pose
    def publish_pose(self, pose, marginals):
        # Define msg
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "odom"

        # Add position
        pose_msg.pose.pose.position.x = pose.x()
        pose_msg.pose.pose.position.y = pose.y()
        pose_msg.pose.pose.position.z = pose.z()

        # Convert rotation matrix to quaternion
        rotation_matrix = pose.rotation().matrix()
        nowa = np.eye(4)
        nowa[:3, :3] = rotation_matrix
        quat = tf_transformations.quaternion_from_matrix(nowa)

        # Add orientation
        pose_msg.pose.pose.orientation.w = quat[3]
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]

        # Create covariance matrix
        cov = marginals.marginalCovariance(sh.O(self.key-1))
        pose_msg.pose.covariance = [
            cov[0, 0], cov[0, 1], cov[0, 2], cov[0, 3], cov[0, 4], cov[0, 5],
            cov[1, 0], cov[1, 1], cov[1, 2], cov[1, 3], cov[1, 4], cov[1, 5],
            cov[2, 0], cov[2, 1], cov[2, 2], cov[2, 3], cov[2, 4], cov[2, 5],
            cov[3, 0], cov[3, 1], cov[3, 2], cov[3, 3], cov[3, 4], cov[3, 5],
            cov[4, 0], cov[4, 1], cov[4, 2], cov[4, 3], cov[4, 4], cov[4, 5],
            cov[5, 0], cov[5, 1], cov[5, 2], cov[5, 3], cov[5, 4], cov[5, 5]
        ]

        # Publish pose
        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()