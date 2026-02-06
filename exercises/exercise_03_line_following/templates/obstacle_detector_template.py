#!/usr/bin/env python3
"""
Hinderniserkennung mit LiDAR

Dieses Template wertet LiDAR-Daten aus und erkennt Hindernisse
im Fahrbereich des Roboters.

Aufgabe B.3: Vervollständigen Sie die TODOs
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')

        # Parameter deklarieren
        self.declare_parameter('detection_angle', 30.0)  # Winkel in Grad (beidseitig)
        self.declare_parameter('stop_distance', 0.35)    # Stopp-Distanz in Metern
        self.declare_parameter('min_valid_range', 0.1)   # Minimale gültige Messung

        # Parameter laden
        self.detection_angle = self.get_parameter('detection_angle').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.min_valid_range = self.get_parameter('min_valid_range').value

        # In Radiant umrechnen
        self.detection_angle_rad = math.radians(self.detection_angle)

        # Subscriber für LiDAR-Daten
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher für Hindernis-Status
        self.obstacle_pub = self.create_publisher(
            Bool,
            '/obstacle_detected',
            10
        )

        self.get_logger().info('Obstacle Detector Node gestartet')
        self.get_logger().info(f'  Prüfwinkel: +/- {self.detection_angle}°')
        self.get_logger().info(f'  Stopp-Distanz: {self.stop_distance} m')

    def scan_callback(self, msg: LaserScan):
        """
        Callback für eingehende LiDAR-Scans.

        TODO 1: Implementieren Sie die Hinderniserkennung

        LaserScan Message:
        - msg.ranges[]: Array mit Distanzen (in Metern)
        - msg.angle_min: Startwinkel (typisch: -π = hinten links)
        - msg.angle_max: Endwinkel (typisch: +π = hinten rechts)
        - msg.angle_increment: Winkel zwischen Messungen

        TurtleBot 3 LDS-01:
        - 360° Scan
        - Index 0 = hinten, Index 180 = vorne (ca.)
        - Auflösung: ~1° pro Messung

        Strategie:
        1. Berechne welche Indizes dem Frontalbereich entsprechen
        2. Extrahiere diese Messungen
        3. Prüfe ob Minimum < stop_distance
        """

        # ===== TODO: Relevante Indizes berechnen =====

        # Anzahl der Messungen
        num_ranges = len(msg.ranges)

        # Winkel pro Messung
        angle_increment = msg.angle_increment

        # Index der Vorwärtsrichtung (0°)
        # Bei TurtleBot: 0° = vorne
        # msg.angle_min ist typischerweise 0 oder -π

        # TODO: Berechne Start- und End-Index für den Prüfbereich
        # Hinweis: Der Bereich geht von -detection_angle bis +detection_angle

        # Winkel zu Index umrechnen
        # index = (winkel - angle_min) / angle_increment

        front_index = int((0 - msg.angle_min) / angle_increment) % num_ranges

        # Anzahl Messungen im Prüfbereich (pro Seite)
        index_range = int(self.detection_angle_rad / angle_increment)

        # Start- und End-Index (mit Wraparound für 360°-Scan)
        start_index = (front_index - index_range) % num_ranges
        end_index = (front_index + index_range) % num_ranges

        # ===== TODO: Messungen im Prüfbereich extrahieren =====

        relevant_ranges = []

        # TODO: Extrahiere die relevanten Distanzen
        # Beachte: Bei 360°-Scan kann start_index > end_index sein!

        if start_index <= end_index:
            # Normaler Fall: zusammenhängender Bereich
            relevant_ranges = list(msg.ranges[start_index:end_index + 1])
        else:
            # Wraparound: Bereich geht über Index 0
            relevant_ranges = list(msg.ranges[start_index:]) + list(msg.ranges[:end_index + 1])

        # ===== TODO: Hindernis prüfen =====

        obstacle_detected = self.check_obstacle(relevant_ranges)

        # Status publizieren
        obstacle_msg = Bool()
        obstacle_msg.data = obstacle_detected
        self.obstacle_pub.publish(obstacle_msg)

        # Debug-Ausgabe
        if obstacle_detected:
            min_dist = min([r for r in relevant_ranges
                          if r > self.min_valid_range and not math.isinf(r)],
                          default=float('inf'))
            self.get_logger().info(
                f'HINDERNIS erkannt! Distanz: {min_dist:.2f} m',
                throttle_duration_sec=0.5
            )

    def check_obstacle(self, ranges: list) -> bool:
        """
        Prüft ob ein Hindernis im Messbereich ist.

        TODO 2: Implementieren Sie die Prüflogik

        Args:
            ranges: Liste von Distanzmessungen im Prüfbereich

        Returns:
            bool: True wenn Hindernis erkannt
        """
        if not ranges:
            return False

        # TODO: Implementieren Sie die Hinderniserkennung
        #
        # Tipps:
        # - Ungültige Messungen filtern (inf, nan, < min_valid_range)
        # - Minimum der gültigen Messungen finden
        # - Mit stop_distance vergleichen

        for distance in ranges:
            # Ungültige Werte überspringen
            if math.isnan(distance) or math.isinf(distance):
                continue

            # Zu nahe Messungen ignorieren (Sensorrauschen)
            if distance < self.min_valid_range:
                continue

            # TODO: Prüfen ob Distanz < Stopp-Distanz
            # if distance < self.stop_distance:
            #     return True
            pass  # TODO: Ersetzen

        return False


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
