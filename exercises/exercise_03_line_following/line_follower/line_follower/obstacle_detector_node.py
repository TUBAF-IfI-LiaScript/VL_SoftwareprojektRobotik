#!/usr/bin/env python3
"""
Hinderniserkennung mit LiDAR

Kopieren Sie den Inhalt von templates/obstacle_detector_template.py hierher
und vervollständigen Sie die TODOs.

Aufgabe B.3
"""

# TODO: Kopieren Sie das Template und implementieren Sie die Hinderniserkennung

import rclpy
from rclpy.node import Node


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')
        self.get_logger().error('NICHT IMPLEMENTIERT!')
        self.get_logger().error('Kopieren Sie das Template aus templates/obstacle_detector_template.py')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
