# cloud_processing/ground_detection_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
from sklearn.linear_model import RANSACRegressor

class GroundDetectionNode(Node):
    def __init__(self):
        super().__init__('ground_detection_node')

        self.declare_parameter('ground_threshold', 0.05)  # Abstand zur Ebene in Metern
        self.declare_parameter('min_ground_points', 100)   # Mindestanzahl Punkte für Boden
        
        self.ground_threshold = self.get_parameter('ground_threshold').value
        self.min_ground_points = self.get_parameter('min_ground_points').value

        self.sub = self.create_subscription(
            PointCloud2,
            'filtered_cloud',
            self.callback,
            10)

        self.ground_pub = self.create_publisher(
            PointCloud2,
            'ground_cloud',
            10)
            
        self.obstacle_pub = self.create_publisher(
            PointCloud2,
            'obstacle_cloud',
            10)

    def callback(self, msg: PointCloud2):
        cloud = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        pts = cloud[:, 0:12].view(np.float32).reshape(-1, 3)
        
        if pts.shape[0] < self.min_ground_points:
            return

        # Downsampling für Performance
        sample_indices = np.arange(0, len(pts), 3)
        pts_sample = pts[sample_indices]
        
        # RANSAC Ebenenfit für Bodenerkennung
        # Verwende X,Z Koordinaten (Y ist oft die Höhe)
        X = pts_sample[:, [0, 2]]  # X,Z für horizontale Ebene
        y = pts_sample[:, 1]       # Y als Höhe
        
        # RANSAC für robuste Ebenenfit
        ransac = RANSACRegressor(
            residual_threshold=self.ground_threshold,
            min_samples=50,
            max_trials=100,
            random_state=42
        )
        
        try:
            ransac.fit(X, y)
            
            # Vorhersage für alle Punkte
            X_all = pts[:, [0, 2]]
            y_pred = ransac.predict(X_all)
            
            # Klassifikation: Boden vs. Hindernisse
            distances = np.abs(pts[:, 1] - y_pred)
            ground_mask = distances < self.ground_threshold
            obstacle_mask = ~ground_mask
            
            # Ground Points
            ground_cloud = cloud[ground_mask]
            if ground_cloud.shape[0] > 0:
                ground_msg = self.create_pointcloud_msg(msg, ground_cloud)
                self.ground_pub.publish(ground_msg)
            
            # Obstacle Points  
            obstacle_cloud = cloud[obstacle_mask]
            if obstacle_cloud.shape[0] > 0:
                obstacle_msg = self.create_pointcloud_msg(msg, obstacle_cloud)
                self.obstacle_pub.publish(obstacle_msg)
                
            self.get_logger().info(f'Ground: {ground_cloud.shape[0]}, Obstacles: {obstacle_cloud.shape[0]}')
            
        except Exception as e:
            self.get_logger().warn(f'Ground detection failed: {e}')

    def create_pointcloud_msg(self, original_msg, cloud_data):
        out = PointCloud2()
        out.header = original_msg.header
        out.height = 1
        out.width = cloud_data.shape[0]
        out.fields = original_msg.fields
        out.is_bigendian = original_msg.is_bigendian
        out.point_step = original_msg.point_step
        out.row_step = original_msg.point_step * cloud_data.shape[0]
        out.is_dense = True
        out.data = cloud_data.tobytes()
        return out


def main(args=None):
    rclpy.init(args=args)
    node = GroundDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
