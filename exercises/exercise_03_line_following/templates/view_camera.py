#!/usr/bin/env python3
"""
Einfaches Skript zum Anzeigen des Kamerabildes.

Nützlich für:
- Verbindungstest
- Schwellwert-Kalibrierung
- Debugging der Linienerkennung

Verwendung:
    python3 view_camera.py

Tasten:
    q - Beenden
    s - Screenshot speichern
    t - Threshold-Modus umschalten
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        self.bridge = CvBridge()
        self.show_threshold = False
        self.threshold_value = 200
        self.screenshot_counter = 0

        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Camera Viewer gestartet')
        self.get_logger().info('Tasten: q=Beenden, s=Screenshot, t=Threshold, +/-=Schwellwert')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            display_image = cv_image.copy()

            if self.show_threshold:
                # Graustufen
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

                # Schwellwert
                _, binary = cv2.threshold(gray, self.threshold_value, 255, cv2.THRESH_BINARY)

                # In 3-Kanal konvertieren für Anzeige
                display_image = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

                # Schwellwert anzeigen
                cv2.putText(display_image, f'Threshold: {self.threshold_value}',
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # ROI-Linie einzeichnen (60% von oben)
            height = display_image.shape[0]
            roi_y = int(height * 0.6)
            cv2.line(display_image, (0, roi_y), (display_image.shape[1], roi_y),
                    (0, 255, 0), 1)
            cv2.putText(display_image, 'ROI', (10, roi_y - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # Bildmitte markieren
            cx = display_image.shape[1] // 2
            cv2.line(display_image, (cx, roi_y), (cx, height), (255, 0, 0), 1)

            cv2.imshow('Camera View', display_image)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                self.get_logger().info('Beende...')
                rclpy.shutdown()
            elif key == ord('s'):
                filename = f'screenshot_{self.screenshot_counter:03d}.png'
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f'Screenshot gespeichert: {filename}')
                self.screenshot_counter += 1
            elif key == ord('t'):
                self.show_threshold = not self.show_threshold
                mode = 'Threshold' if self.show_threshold else 'Normal'
                self.get_logger().info(f'Modus: {mode}')
            elif key == ord('+') or key == ord('='):
                self.threshold_value = min(255, self.threshold_value + 5)
                self.get_logger().info(f'Threshold: {self.threshold_value}')
            elif key == ord('-'):
                self.threshold_value = max(0, self.threshold_value - 5)
                self.get_logger().info(f'Threshold: {self.threshold_value}')

        except Exception as e:
            self.get_logger().error(f'Fehler: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
