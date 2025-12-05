#!/usr/bin/env python3
"""
Face Detection Node for ROS2

Subscribed Topics:
    /image_raw (sensor_msgs/Image): Input image from camera

Published Topics:
    /face_detection/image (sensor_msgs/Image): Image with face detection boxes
    /face_detection/faces (std_msgs/Int32): Number of detected faces

Author: Sebastian Zug
License: MIT
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2


class FaceDetectorNode(Node):
    """
    ROS2 Node für Gesichtserkennung mit OpenCV Haar Cascade
    """

    def __init__(self):
        super().__init__('face_detector_node')

        # Parameter deklarieren
        self.declare_parameter('scale_factor', 1.1)
        self.declare_parameter('min_neighbors', 5)
        self.declare_parameter('min_size_width', 30)
        self.declare_parameter('min_size_height', 30)
        self.declare_parameter('cascade_file', '')

        # Parameter auslesen
        self.scale_factor = self.get_parameter('scale_factor').value
        self.min_neighbors = self.get_parameter('min_neighbors').value
        self.min_size_w = self.get_parameter('min_size_width').value
        self.min_size_h = self.get_parameter('min_size_height').value
        cascade_file = self.get_parameter('cascade_file').value

        # CvBridge für ROS <-> OpenCV Konvertierung
        self.bridge = CvBridge()

        # Lade Haar Cascade Classifier
        if cascade_file == '':
            # Standard OpenCV Cascade - versuche verschiedene Pfade
            import os
            possible_paths = [
                # OpenCV Python package (neuere Versionen)
                getattr(cv2.data, 'haarcascades', '') + 'haarcascade_frontalface_default.xml' if hasattr(cv2, 'data') else '',
                # System-Installation (Ubuntu/Debian)
                '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
                '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml',
                # Fallback: suche im Python-Paket
                os.path.join(os.path.dirname(cv2.__file__), 'data', 'haarcascade_frontalface_default.xml'),
            ]

            cascade_file = None
            for path in possible_paths:
                if path and os.path.exists(path):
                    cascade_file = path
                    break

            if not cascade_file:
                self.get_logger().error('Konnte keinen Haar Cascade finden!')
                self.get_logger().error('Mögliche Pfade: ' + str(possible_paths))
                raise RuntimeError('Haar Cascade nicht gefunden!')

            self.get_logger().info(f'Nutze Haar Cascade: {cascade_file}')

        self.face_cascade = cv2.CascadeClassifier(cascade_file)

        if self.face_cascade.empty():
            self.get_logger().error(f'Konnte Haar Cascade nicht laden: {cascade_file}')
            raise RuntimeError('Haar Cascade konnte nicht geladen werden!')

        self.get_logger().info('Haar Cascade erfolgreich geladen')

        # QoS-Profile für Sensor-Daten (kompatibel mit v4l2_camera)
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        publisher_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber für Kamera-Bilder (mit sensor_qos für Kompatibilität)
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            sensor_qos
        )

        # Publisher für annotiertes Bild
        self.image_pub = self.create_publisher(
            Image,
            '/face_detection/image',
            publisher_qos
        )

        # Publisher für Anzahl erkannter Gesichter
        self.faces_pub = self.create_publisher(
            Int32,
            '/face_detection/faces',
            publisher_qos
        )

        self.get_logger().info('Face Detector Node gestartet')
        self.get_logger().info(f'Parameter: scale_factor={self.scale_factor}, '
                              f'min_neighbors={self.min_neighbors}, '
                              f'min_size=({self.min_size_w}, {self.min_size_h})')

    def image_callback(self, msg):
        """
        Callback-Funktion für eingehende Bilder

        Args:
            msg (sensor_msgs/Image): Eingangsbild
        """
        try:
            # ROS Image → OpenCV Image (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Gesichtserkennung durchführen
            faces = self.detect_faces(cv_image)

            # Zeichne Rechtecke um erkannte Gesichter
            annotated_image = self.draw_faces(cv_image, faces)

            # Konvertiere zurück zu ROS Image
            output_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            output_msg.header = msg.header  # Zeitstempel übernehmen

            # Publiziere annotiertes Bild
            self.image_pub.publish(output_msg)

            # Publiziere Anzahl erkannter Gesichter
            face_count_msg = Int32()
            face_count_msg.data = len(faces)
            self.faces_pub.publish(face_count_msg)

            if len(faces) > 0:
                self.get_logger().info(f'{len(faces)} Gesicht(er) erkannt', throttle_duration_sec=1.0)

        except Exception as e:
            self.get_logger().error(f'Fehler bei Bildverarbeitung: {str(e)}')

    def detect_faces(self, image):
        """
        Erkennt Gesichter im Bild mit Haar Cascade

        Args:
            image (numpy.ndarray): Eingangsbild (BGR)

        Returns:
            list: Liste von Gesichts-Rechtecken [(x, y, w, h), ...]
        """
        # Konvertiere zu Graustufen (für bessere Performance)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Optional: Bild verkleinern für schnellere Verarbeitung
        # scale_down = 0.5  # 50% kleiner
        # gray_small = cv2.resize(gray, None, fx=scale_down, fy=scale_down)
        # ... dann Ergebnisse zurückskalieren

        # Gesichtserkennung
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbors,
            minSize=(self.min_size_w, self.min_size_h)
        )

        return faces

    def draw_faces(self, image, faces):
        """
        Zeichnet Rechtecke um erkannte Gesichter

        Args:
            image (numpy.ndarray): Eingangsbild
            faces (list): Liste von Gesichts-Rechtecken

        Returns:
            numpy.ndarray: Annotiertes Bild
        """
        output_image = image.copy()

        for (x, y, w, h) in faces:
            # Zeichne Rechteck um Gesicht (grün)
            cv2.rectangle(output_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Optional: Füge Text hinzu
            cv2.putText(
                output_image,
                'Face',
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

        # Zeige Gesamtanzahl der Gesichter
        text = f'Faces: {len(faces)}'
        cv2.putText(
            output_image,
            text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2
        )

        return output_image


def main(args=None):
    """Hauptfunktion"""
    rclpy.init(args=args)

    try:
        node = FaceDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Fehler: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
