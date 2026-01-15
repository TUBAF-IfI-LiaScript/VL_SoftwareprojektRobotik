#!/usr/bin/env python3
"""
ZED-Objekterkennungs-Extraktor Node für Übung 2

Dieser ROS 2 Node extrahiert die Personendetektionen aus den ZED-Objektdaten
und speichert sie als CSV-Datei. Diese dient als Ground Truth für den
Vergleich mit YOLOv8.

Verwendung:
    # Terminal 1: Bag abspielen
    ros2 bag play data/20251126_ifi2

    # Terminal 2: ZED-Extraktor starten
    ros2 launch person_detector zed_extractor.launch.py

    # Oder direkt:
    ros2 run person_detector zed_extractor_node

    # Ctrl+C zum Beenden - Ergebnisse werden automatisch gespeichert
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import csv
import os
import sys

# ZED Messages Import
try:
    from zed_msgs.msg import ObjectsStamped
    ZED_MSGS_AVAILABLE = True
except ImportError:
    ZED_MSGS_AVAILABLE = False
    print("=" * 60)
    print("FEHLER: zed_msgs nicht installiert!")
    print()
    print("Installieren Sie zed_msgs mit:")
    print("  sudo apt install ros-jazzy-zed-msgs")
    print()
    print("Oder bauen Sie es aus dem Source:")
    print("  https://github.com/stereolabs/zed-ros2-wrapper")
    print("=" * 60)
    sys.exit(1)


class ZedExtractorNode(Node):
    """
    Extrahiert Personendetektionen aus ZED ObjectsStamped Messages.
    """

    def __init__(self):
        super().__init__('zed_extractor')

        self.declare_parameter('output_csv', 'results/zed_detections.csv')
        self.output_csv = self.get_parameter('output_csv').value

        # Ergebnisse speichern
        self.results = []
        self.frame_count = 0

        # QoS Profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber
        self.subscription = self.create_subscription(
            ObjectsStamped,
            '/zed2i_front/zed_node_front/obj_det/objects',
            self.callback,
            qos
        )

        self.get_logger().info('ZED Extractor gestartet.')
        self.get_logger().info('Warte auf ZED-Objektdaten...')
        self.get_logger().info('Drücken Sie Ctrl+C zum Beenden und Speichern.')

    def callback(self, msg: ObjectsStamped):
        """Verarbeite eingehende ZED-Objektdaten."""
        self.frame_count += 1

        # Timestamp extrahieren
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Personen zählen
        person_count = 0
        person_details = []

        for obj in msg.objects:
            # Prüfe auf Person (Label oder ID)
            # ZED verwendet typischerweise label_id oder label string
            is_person = False

            # Verschiedene Möglichkeiten prüfen
            if hasattr(obj, 'label') and obj.label.lower() == 'person':
                is_person = True
            elif hasattr(obj, 'label_id') and obj.label_id == 0:
                is_person = True
            elif hasattr(obj, 'sublabel') and 'person' in obj.sublabel.lower():
                is_person = True

            if is_person:
                person_count += 1

                # Details extrahieren (falls verfügbar)
                detail = {'id': getattr(obj, 'tracking_id', -1)}

                if hasattr(obj, 'confidence'):
                    detail['confidence'] = obj.confidence

                if hasattr(obj, 'position'):
                    # Position kann ein Array [x,y,z] oder ein Objekt mit .x/.y/.z sein
                    pos = obj.position
                    if hasattr(pos, 'x'):
                        detail['x'] = pos.x
                        detail['y'] = pos.y
                        detail['z'] = pos.z
                    elif hasattr(pos, '__getitem__'):
                        detail['x'] = float(pos[0])
                        detail['y'] = float(pos[1])
                        detail['z'] = float(pos[2])

                person_details.append(detail)

        # Ergebnis speichern
        self.results.append({
            'timestamp': timestamp,
            'person_count': person_count,
            'details': person_details
        })

        # Logging
        if self.frame_count % 50 == 0:
            self.get_logger().info(
                f'Frame {self.frame_count}: {person_count} Personen erkannt'
            )

    def save_results(self):
        """Speichere Ergebnisse als CSV."""
        if not self.results:
            self.get_logger().warn('Keine Daten zum Speichern!')
            return

        # Verzeichnis erstellen
        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)

        # CSV schreiben
        with open(self.output_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'zed_person_count'])

            for result in self.results:
                writer.writerow([
                    f"{result['timestamp']:.6f}",
                    result['person_count']
                ])

        self.get_logger().info(f'Gespeichert: {len(self.results)} Frames')
        self.get_logger().info(f'Ausgabedatei: {self.output_csv}')

        # Statistiken
        counts = [r['person_count'] for r in self.results]
        avg = sum(counts) / len(counts)
        max_count = max(counts)

        self.get_logger().info('-' * 40)
        self.get_logger().info(f'Durchschnitt: {avg:.2f} Personen/Frame')
        self.get_logger().info(f'Maximum: {max_count} Personen')
        self.get_logger().info(f'Frames mit Personen: {sum(1 for c in counts if c > 0)}')


def main(args=None):
    rclpy.init(args=args)

    node = ZedExtractorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Beende und speichere...')
    finally:
        node.save_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
