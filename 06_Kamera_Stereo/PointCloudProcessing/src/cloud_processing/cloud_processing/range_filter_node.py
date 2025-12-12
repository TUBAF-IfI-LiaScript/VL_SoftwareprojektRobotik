# cloud_processing/range_filter_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct

class RangeFilterNode(Node):
    def __init__(self):
        super().__init__('range_filter_node')

        self.declare_parameter('min_dist', 0.5)
        self.declare_parameter('max_dist', 8.0)
        self.declare_parameter('input_topic', '/zed2i_front/zed_node_front/point_cloud/cloud_registered')

        input_topic = self.get_parameter('input_topic').value

        self.sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.callback,
            10)

        self.pub = self.create_publisher(
            PointCloud2,
            'filtered_cloud',
            10)

        self.get_logger().info(f"RangeFilter subscribed to {input_topic}")

    def callback(self, msg: PointCloud2):

        min_dist = self.get_parameter('min_dist').value
        max_dist = self.get_parameter('max_dist').value

        pts = np.frombuffer(msg.data, dtype=np.uint8)
        cloud = pts.reshape(-1, msg.point_step)

        # Extract XYZ fields from cloud
        # Field offsets are always 0,4,8 for x,y,z in ZED clouds
        x = cloud[:, 0:4].view(np.float32).flatten()
        y = cloud[:, 4:8].view(np.float32).flatten()
        z = cloud[:, 8:12].view(np.float32).flatten()

        d = np.sqrt(x*x + y*y + z*z)

        mask = (d > min_dist) & (d < max_dist)

        filtered = cloud[mask]

        out_msg = PointCloud2()
        out_msg.header = msg.header
        out_msg.height = 1
        out_msg.width = filtered.shape[0]
        out_msg.fields = msg.fields
        out_msg.is_bigendian = msg.is_bigendian
        out_msg.point_step = msg.point_step
        out_msg.row_step = msg.point_step * filtered.shape[0]
        out_msg.is_dense = True
        out_msg.data = filtered.tobytes()

        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RangeFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
