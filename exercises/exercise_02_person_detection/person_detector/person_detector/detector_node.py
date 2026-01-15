#!/usr/bin/env python3
"""
YOLO Personendetektor Node - Übung 2

AUFGABE 2: Implementieren Sie die Personenzählung mit YOLOv8.

Dieses Template enthält die Grundstruktur für die YOLOv8-Personendetektion.
Sie müssen nur die Funktion count_persons_yolo() implementieren.

Architektur:
    - Subscriber: RGB-Bild für YOLOv8-Inferenz
    - Zählung der erkannten Personen pro Frame
    - CSV-Export der Ergebnisse

Lernziele:
    - YOLOv8-Inferenz mit ultralytics
    - ROS2 Image zu OpenCV Konvertierung (cv_bridge)
    - QoS-Profile verstehen (siehe Kommentare unten)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import csv
import os
import time

# YOLOv8 Import
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("WARNING: ultralytics not installed. Run: pip install ultralytics")


class PersonDetectorNode(Node):
    """
    ROS 2 Node zur Personendetektion mit YOLOv8.

    Subscriber:
        - /zed2i_front/zed_node_front/rgb/color/rect/image (sensor_msgs/Image)

    Output:
        - CSV-Datei mit YOLO-Detektionsergebnissen
    """

    def __init__(self):
        super().__init__('person_detector')

        # === Parameter deklarieren ===
        self.declare_parameter('model_name', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('output_csv', 'results/yolo_detections.csv')
        self.declare_parameter('save_images', False)

        # Parameter auslesen
        model_name = self.get_parameter('model_name').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.output_csv = self.get_parameter('output_csv').value

        # save_images: String 'true'/'false' aus Launch-File zu Bool konvertieren
        save_images_param = self.get_parameter('save_images').value
        if isinstance(save_images_param, str):
            self.save_images = save_images_param.lower() == 'true'
        else:
            self.save_images = bool(save_images_param)

        self.get_logger().info(f'Starting PersonDetector with model: {model_name}')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')

        # === YOLOv8 Modell laden ===
        if YOLO_AVAILABLE:
            self.get_logger().info('Loading YOLOv8 model...')
            self.model = YOLO(model_name)
            self.get_logger().info('YOLOv8 model loaded successfully')
        else:
            self.model = None
            self.get_logger().error('YOLOv8 not available!')

        # === CV Bridge für Bildkonvertierung ===
        self.bridge = CvBridge()

        # === Ergebnisse speichern ===
        self.results = []
        self.frame_count = 0

        # === Framerate-Messung ===
        self.start_time = None
        self.processing_times = []

        # =====================================================================
        # QoS Profile - WICHTIG FÜR AUFGABE 2.2
        # =====================================================================
        # QoS (Quality of Service) steuert, wie Nachrichten übertragen werden.
        #
        # RELIABILITY:
        #   - RELIABLE: Garantiert Zustellung (mit Retransmission)
        #   - BEST_EFFORT: Keine Garantie, schneller (für Sensordaten üblich)
        #
        # HISTORY:
        #   - KEEP_LAST: Nur die letzten N Nachrichten speichern
        #   - KEEP_ALL: Alle Nachrichten speichern
        #
        # DEPTH:
        #   - Anzahl der Nachrichten im Puffer
        #   - Bei langsamer Verarbeitung (YOLO ~150ms) sammeln sich Bilder an!
        #
        # PROBLEM: Wenn YOLO langsamer als die Bildrate ist (30 FPS = 33ms),
        # füllt sich der Puffer. Mit depth=10 werden alte Bilder verarbeitet,
        # obwohl neuere bereits verfügbar sind.
        #
        # AUFGABE 2.2: Experimentieren Sie mit depth=1 vs depth=10
        # Was beobachten Sie bezüglich der verarbeiteten Frames?
        # =====================================================================
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # AUFGABE: Testen Sie auch depth=10 und vergleichen Sie
        )

        # === Image Subscriber ===
        self.image_sub = self.create_subscription(
            Image,
            '/zed2i_front/zed_node_front/rgb/color/rect/image',
            self.image_callback,
            qos
        )

        self.get_logger().info('PersonDetector initialized and waiting for images...')

    def image_callback(self, image_msg: Image):
        """
        Callback für eingehende RGB-Bilder.

        Args:
            image_msg: RGB-Bild als sensor_msgs/Image
        """
        # Startzeit für Gesamtmessung
        if self.start_time is None:
            self.start_time = time.time()

        self.frame_count += 1

        # === Framerate-Messung Start ===
        process_start = time.time()

        # === 1. YOLO-Personenzählung ===
        # count_persons_yolo gibt ein Tuple zurück: (anzahl, liste_der_boxes)
        yolo_count, person_boxes = self.count_persons_yolo(image_msg)

        # === Framerate-Messung Ende ===
        process_time = time.time() - process_start
        self.processing_times.append(process_time)

        # === 2. Timestamp extrahieren ===
        timestamp = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9

        # === 3. Ergebnis speichern ===
        self.results.append({
            'timestamp': timestamp,
            'yolo_count': yolo_count
        })

        # === 4. Logging mit Framerate ===
        if self.frame_count % 10 == 0:
            avg_time = sum(self.processing_times[-10:]) / min(10, len(self.processing_times))
            fps = 1.0 / avg_time if avg_time > 0 else 0
            self.get_logger().info(
                f'Frame {self.frame_count:4d} | YOLO: {yolo_count} Personen | '
                f'{process_time*1000:.0f}ms ({fps:.1f} FPS)'
            )

        # === 5. Debug-Bild speichern (optional) ===
        if self.save_images and yolo_count > 0:
            self.save_debug_image(image_msg, person_boxes)

    def count_persons_yolo(self, image_msg: Image) -> tuple:
        """
        Zähle Personen im Bild mittels YOLOv8.

        AUFGABE 2.1: Implementieren Sie diese Funktion!

        Args:
            image_msg: ROS Image Message

        Returns:
            Tuple: (anzahl_personen, liste_der_person_boxes)
            Jede Box ist ein Dict mit: {'xyxy': [x1,y1,x2,y2], 'conf': float}

        Hinweise:
            - COCO class_id 0 = Person
            - Nutzen Sie self.confidence_threshold für die Filterung
        """
        if self.model is None:
            return 0, []

        # TODO (AUFGABE 2.1): Implementieren Sie hier die YOLO-Personenzählung
        # Siehe README.md für Details
        person_count = 0
        person_boxes = []

        return person_count, person_boxes

    def save_debug_image(self, image_msg: Image, person_boxes: list):
        """
        Speichere Bild mit Bounding Boxes für Analyse.

        AUFGABE 2.3 (Optional): Zeichnen Sie Bounding Boxes um erkannte Personen.

        Args:
            image_msg: ROS Image Message
            person_boxes: Liste von Dicts mit {'xyxy': [x1,y1,x2,y2], 'conf': float}
        """
        try:
            import cv2
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')

            # Ordner erstellen falls nicht vorhanden
            os.makedirs('results/debug_images', exist_ok=True)

            # TODO (AUFGABE 2.3): Bounding Boxes zeichnen - Siehe README.md

            filename = f'results/debug_images/frame_{self.frame_count:05d}_yolo{len(person_boxes)}.jpg'
            cv2.imwrite(filename, cv_image)

            self.get_logger().debug(f'Saved debug image: {filename}')

        except Exception as e:
            self.get_logger().error(f'Failed to save debug image: {e}')

    def save_results(self):
        """Speichere Ergebnisse als CSV-Datei."""
        if not self.results:
            self.get_logger().warn('No results to save!')
            return

        # Ordner erstellen falls nicht vorhanden
        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)

        with open(self.output_csv, 'w', newline='') as f:
            fieldnames = ['timestamp', 'yolo_count']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.results)

        self.get_logger().info(f'Saved {len(self.results)} results to {self.output_csv}')

        # Statistiken ausgeben
        self.print_statistics()

    def print_statistics(self):
        """Gebe Zusammenfassung der Ergebnisse aus."""
        if not self.results:
            return

        yolo_counts = [r['yolo_count'] for r in self.results]

        # Einfache Statistiken ohne numpy
        avg = sum(yolo_counts) / len(yolo_counts)
        max_count = max(yolo_counts)
        min_count = min(yolo_counts)
        frames_with_persons = sum(1 for c in yolo_counts if c > 0)

        # Framerate-Statistiken
        total_time = time.time() - self.start_time if self.start_time else 0
        effective_fps = len(self.results) / total_time if total_time > 0 else 0
        avg_process_time = sum(self.processing_times) / len(self.processing_times) if self.processing_times else 0

        self.get_logger().info('=' * 60)
        self.get_logger().info('YOLO DETECTION SUMMARY')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Total frames analyzed: {len(self.results)}')
        self.get_logger().info(f'Total time: {total_time:.1f}s')
        self.get_logger().info(f'Effective FPS: {effective_fps:.1f}')
        self.get_logger().info(f'Avg processing time: {avg_process_time*1000:.0f}ms')
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'Average persons/frame: {avg:.2f}')
        self.get_logger().info(f'Min persons: {min_count}')
        self.get_logger().info(f'Max persons: {max_count}')
        self.get_logger().info(f'Frames with persons: {frames_with_persons}')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Results saved to: {self.output_csv}')
        self.get_logger().info('Run analyze_results.py to compare with ZED data')


def main(args=None):
    rclpy.init(args=args)

    node = PersonDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down (Ctrl+C)...')
    finally:
        # Ergebnisse speichern beim Beenden
        node.save_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
