import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import gtsam
from gtsam import symbol_shorthand as sh
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf_transformations
import numpy as np

class SimpleOdometryNode(Node):
    def __init__(self):
        super().__init__('simple_odometry_node')
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'estimated_pose', 10)
        
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()
        self.optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimate)
        
        self.prev_pose = None
        self.key = 0

    def odom_callback(self, msg):
        # Odczytaj pozycję i orientację z wiadomości odometrycznej
        current_pose = gtsam.Pose3(gtsam.Rot3.Quaternion(msg.pose.pose.orientation.w,
                                                         msg.pose.pose.orientation.x,
                                                         msg.pose.pose.orientation.y,
                                                         msg.pose.pose.orientation.z),
                                   gtsam.Point3(msg.pose.pose.position.x,
                                                msg.pose.pose.position.y,
                                                msg.pose.pose.position.z))
        
        if self.prev_pose is None:
            # Pierwsza wiadomość odometryczna: ustaw priorytet początkowy
            self.graph.add(gtsam.PriorFactorPose3(sh.O(self.key), current_pose, gtsam.noiseModel.Isotropic.Sigma(6, 0.1)))
            self.initial_estimate.insert(sh.O(self.key), current_pose)
        else:
            # Dodaj transformację między poprzednią a aktualną pozycją
            odom_noise = gtsam.noiseModel.Isotropic.Sigma(6, 0.01)
            self.key += 1
            odometry_factor = gtsam.BetweenFactorPose3(sh.O(self.key - 1), sh.O(self.key), self.prev_pose.between(current_pose), odom_noise)
            self.graph.add(odometry_factor)
            self.initial_estimate.insert(sh.O(self.key), current_pose)

        self.prev_pose = current_pose
        self.optimize()

    def optimize(self):
        try:
            self.optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimate)
            if self.key > 1:  # Rozpocznij optymalizację od drugiego klucza
                result = self.optimizer.optimize()
                optimized_pose = result.atPose3(sh.O(self.key-1))
                marginals = gtsam.Marginals(self.graph, result)
                # print(optimized_pose)
                self.publish_pose(optimized_pose, marginals)
        except Exception as e:
            print(f"Optimization error: {e}")

    def publish_pose(self, pose, marginals):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "odom"
        pose_msg.pose.pose.position.x = pose.x()
        pose_msg.pose.pose.position.y = pose.y()
        pose_msg.pose.pose.position.z = pose.z()

        rotation_matrix = pose.rotation().matrix()
        nowa = np.eye(4)
        nowa[:3, :3] = rotation_matrix
        quat = tf_transformations.quaternion_from_matrix(nowa)

        pose_msg.pose.pose.orientation.w = quat[3]
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]

        cov = marginals.marginalCovariance(sh.O(self.key-1))

        pose_msg.pose.covariance = [
            cov[0, 0], cov[0, 1], cov[0, 2], cov[0, 3], cov[0, 4], cov[0, 5],
            cov[1, 0], cov[1, 1], cov[1, 2], cov[1, 3], cov[1, 4], cov[1, 5],
            cov[2, 0], cov[2, 1], cov[2, 2], cov[2, 3], cov[2, 4], cov[2, 5],
            cov[3, 0], cov[3, 1], cov[3, 2], cov[3, 3], cov[3, 4], cov[3, 5],
            cov[4, 0], cov[4, 1], cov[4, 2], cov[4, 3], cov[4, 4], cov[4, 5],
            cov[5, 0], cov[5, 1], cov[5, 2], cov[5, 3], cov[5, 4], cov[5, 5]
        ]

        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
