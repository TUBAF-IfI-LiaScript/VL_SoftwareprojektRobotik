#!/usr/bin/env python3
"""
Linienerkennung für TurtleBot 3

Dieses Template erkennt eine weiße Linie auf dunklem Untergrund
und publiziert deren horizontale Position.

Aufgabe B.1: Vervollständigen Sie die TODOs
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineDetectorNode(Node):
    def __init__(self):
        super().__init__('line_detector_node')

        # Parameter deklarieren
        self.declare_parameter('threshold', 200)  # Schwellwert für Weiß-Erkennung
        self.declare_parameter('roi_top_ratio', 0.6)  # Oberer Rand der ROI (60% = untere 40%)
        self.declare_parameter('debug_image', True)  # Debug-Bild publizieren?

        # Parameter laden
        self.threshold = self.get_parameter('threshold').value
        self.roi_top_ratio = self.get_parameter('roi_top_ratio').value
        self.debug_enabled = self.get_parameter('debug_image').value

        # CV Bridge für ROS ↔ OpenCV Konvertierung
        self.bridge = CvBridge()

        # Subscriber für Kamerabild
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher für Linienposition (-1.0 bis +1.0)
        self.position_pub = self.create_publisher(
            Float32,
            '/line_position',
            10
        )

        # Optional: Debug-Bild publizieren
        if self.debug_enabled:
            self.debug_pub = self.create_publisher(
                Image,
                '/line_detector/debug_image',
                10
            )

        self.get_logger().info('Line Detector Node gestartet')
        self.get_logger().info(f'  Schwellwert: {self.threshold}')
        self.get_logger().info(f'  ROI: untere {int((1-self.roi_top_ratio)*100)}% des Bildes')

    def image_callback(self, msg: Image):
        """
        Callback für eingehende Kamerabilder.

        TODO 1: Implementieren Sie die Linienerkennung

        Schritte:
        1. ROS-Image zu OpenCV konvertieren
        2. ROI (Region of Interest) extrahieren
        3. In Graustufen konvertieren
        4. Schwellwert anwenden
        5. Schwerpunkt berechnen
        6. Position normalisieren und publizieren
        """
        try:
            # Schritt 1: ROS-Image zu OpenCV konvertieren
            # Hinweis: self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv_image = None  # TODO: Konvertieren

            if cv_image is None:
                return

            height, width = cv_image.shape[:2]

            # Schritt 2: ROI extrahieren (nur unterer Teil des Bildes)
            # Die Linie ist typischerweise im unteren Bildbereich
            roi_top = int(height * self.roi_top_ratio)
            roi = cv_image[roi_top:height, :]

            # Schritt 3: In Graustufen konvertieren
            # TODO: gray = cv2.cvtColor(...)
            gray = None  # TODO

            # Schritt 4: Schwellwert anwenden (weiße Linie = hell)
            # TODO: _, binary = cv2.threshold(...)
            binary = None  # TODO

            # Schritt 5: Schwerpunkt der weißen Pixel berechnen
            line_position = self.find_line_center(binary, width)

            # Schritt 6: Position publizieren
            if line_position is not None:
                msg_out = Float32()
                msg_out.data = line_position
                self.position_pub.publish(msg_out)

            # Optional: Debug-Bild erstellen und publizieren
            if self.debug_enabled and binary is not None:
                self.publish_debug_image(cv_image, roi_top, binary, line_position, width)

        except Exception as e:
            self.get_logger().error(f'Fehler in image_callback: {e}')

    def find_line_center(self, binary_image, image_width):
        """
        Findet den Schwerpunkt der weißen Linie im Binärbild.

        TODO 2: Implementieren Sie die Schwerpunktberechnung

        Args:
            binary_image: Binärbild (weiß = Linie)
            image_width: Breite des Originalbildes

        Returns:
            float: Normalisierte Position (-1.0 bis +1.0) oder None
        """
        if binary_image is None:
            return None

        # Momente des Binärbildes berechnen
        # Hinweis: cv2.moments(binary_image)
        moments = None  # TODO

        # Schwerpunkt berechnen
        # moments['m00'] = Gesamtfläche (Anzahl weißer Pixel)
        # moments['m10'] = Summe der x-Koordinaten
        # cx = m10 / m00 = x-Koordinate des Schwerpunkts

        if moments is not None and moments['m00'] > 100:  # Mindestfläche
            cx = moments['m10'] / moments['m00']

            # Normalisieren: 0 = linker Rand, width = rechter Rand
            # → -1.0 = ganz links, 0.0 = Mitte, +1.0 = ganz rechts
            normalized = (cx - image_width / 2) / (image_width / 2)

            return float(normalized)

        # Keine Linie gefunden
        return None

    def publish_debug_image(self, original, roi_top, binary, line_pos, width):
        """
        Publiziert ein Debug-Bild mit Visualisierung der Erkennung.

        Optional: Erweitern Sie diese Funktion für besseres Debugging
        """
        debug_image = original.copy()

        # ROI-Bereich markieren
        cv2.line(debug_image, (0, roi_top), (width, roi_top), (0, 255, 0), 2)

        # Linie einzeichnen, wenn gefunden
        if line_pos is not None:
            # Rückrechnung: normalisiert → Pixel
            cx_pixel = int((line_pos + 1) / 2 * width)
            cv2.line(debug_image, (cx_pixel, roi_top), (cx_pixel, original.shape[0]), (0, 0, 255), 3)

            # Bildmitte als Referenz
            cv2.line(debug_image, (width // 2, roi_top), (width // 2, original.shape[0]), (255, 0, 0), 1)

            # Position als Text
            cv2.putText(debug_image, f'Pos: {line_pos:.2f}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cv2.putText(debug_image, 'Keine Linie!', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Publizieren
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
        self.debug_pub.publish(debug_msg)


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
