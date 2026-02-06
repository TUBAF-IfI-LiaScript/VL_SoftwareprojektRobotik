#!/usr/bin/env python3
"""
PID-Regler für Linienverfolgung

Kopieren Sie den Inhalt von templates/pid_controller_template.py hierher
und vervollständigen Sie die TODOs.

Aufgabe B.2
"""

# TODO: Kopieren Sie das Template und implementieren Sie den PID-Regler

import rclpy
from rclpy.node import Node


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().error('NICHT IMPLEMENTIERT!')
        self.get_logger().error('Kopieren Sie das Template aus templates/pid_controller_template.py')


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
