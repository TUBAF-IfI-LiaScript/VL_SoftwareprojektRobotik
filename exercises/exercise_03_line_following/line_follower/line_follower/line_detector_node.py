#!/usr/bin/env python3
"""
Linienerkennung für TurtleBot 3

Kopieren Sie den Inhalt von templates/line_detector_template.py hierher
und vervollständigen Sie die TODOs.

Aufgabe B.1
"""

# TODO: Kopieren Sie das Template und implementieren Sie die Linienerkennung

import rclpy
from rclpy.node import Node


class LineDetectorNode(Node):
    def __init__(self):
        super().__init__('line_detector_node')
        self.get_logger().error('NICHT IMPLEMENTIERT!')
        self.get_logger().error('Kopieren Sie das Template aus templates/line_detector_template.py')


def main(args=None):
    rclpy.init(args=args)
    node = LineDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
