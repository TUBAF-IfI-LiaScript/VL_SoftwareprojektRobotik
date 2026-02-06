#!/usr/bin/env python3
"""
PID-Regler für Linienverfolgung

Dieses Template implementiert einen PID-Regler, der die Linienposition
in Lenkbefehle (angular.z) umwandelt.

Aufgabe B.2: Vervollständigen Sie die TODOs

Theorie aus VL 10:
    u[k] = Kp * e[k] + Ki * sum(e[i]) * dt + Kd * (e[k] - e[k-1]) / dt

    e[k]  = Fehler (Sollwert - Istwert) = 0 - line_position
    u[k]  = Stellgröße (Winkelgeschwindigkeit omega)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import time


class PIDController:
    """
    Einfache PID-Regler Implementierung.

    TODO 1: Vervollständigen Sie die update()-Methode
    """

    def __init__(self, kp: float, ki: float, kd: float, dt: float = 0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        # Interne Zustände
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None

    def update(self, error: float) -> float:
        """
        Berechnet die Stellgröße basierend auf dem aktuellen Fehler.

        Args:
            error: Regeldifferenz (Sollwert - Istwert)

        Returns:
            float: Stellgröße u
        """
        # Zeitschritt berechnen
        current_time = time.time()
        if self.last_time is not None:
            self.dt = current_time - self.last_time
        self.last_time = current_time

        # Schutz vor Division durch Null
        if self.dt <= 0:
            self.dt = 0.1

        # ===== TODO: PID-Berechnung implementieren =====

        # P-Anteil: proportional zum aktuellen Fehler
        p_term = 0.0  # TODO: self.kp * error

        # I-Anteil: Integral des Fehlers über die Zeit
        # Hinweis: self.integral += error * self.dt
        i_term = 0.0  # TODO

        # D-Anteil: Ableitung des Fehlers
        # Hinweis: (error - self.prev_error) / self.dt
        d_term = 0.0  # TODO

        # Fehler für nächste Iteration speichern
        self.prev_error = error

        # Stellgröße berechnen
        output = p_term + i_term + d_term

        return output

    def reset(self):
        """Setzt den internen Zustand zurück."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Parameter deklarieren
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('linear_velocity', 0.1)  # Vorwärtsgeschwindigkeit [m/s]
        self.declare_parameter('max_angular_velocity', 1.0)  # Max. Drehrate [rad/s]

        # Parameter laden
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value

        # PID-Regler initialisieren
        self.pid = PIDController(kp, ki, kd)

        # Zustand
        self.line_position = None
        self.obstacle_detected = False
        self.last_line_time = None

        # Subscriber
        self.line_sub = self.create_subscription(
            Float32,
            '/line_position',
            self.line_position_callback,
            10
        )

        self.obstacle_sub = self.create_subscription(
            Bool,
            '/obstacle_detected',
            self.obstacle_callback,
            10
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Timer für regelmäßige Regelung (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Controller Node gestartet')
        self.get_logger().info(f'  PID: Kp={kp}, Ki={ki}, Kd={kd}')
        self.get_logger().info(f'  Linear velocity: {self.linear_vel} m/s')

    def line_position_callback(self, msg: Float32):
        """
        Empfängt die Linienposition vom line_detector_node.

        TODO 2: Speichern Sie die Position und den Zeitstempel
        """
        self.line_position = msg.data
        self.last_line_time = self.get_clock().now()

    def obstacle_callback(self, msg: Bool):
        """Empfängt den Hindernis-Status."""
        self.obstacle_detected = msg.data

    def control_loop(self):
        """
        Haupt-Regelschleife, wird periodisch aufgerufen.

        TODO 3: Implementieren Sie die Regellogik

        Ablauf:
        1. Prüfen ob Hindernis → Stopp
        2. Prüfen ob Linie vorhanden → sonst Stopp
        3. Fehler berechnen (Sollwert = 0, Istwert = line_position)
        4. PID-Regler aufrufen
        5. Geschwindigkeitsbefehle publizieren
        """
        twist = Twist()

        # ===== Zustandsprüfungen =====

        # 1. Hindernis erkannt → Stopp
        if self.obstacle_detected:
            self.get_logger().info('Hindernis erkannt - STOPP', throttle_duration_sec=1.0)
            self.publish_stop()
            self.pid.reset()  # PID zurücksetzen während Stopp
            return

        # 2. Keine Liniendaten vorhanden
        if self.line_position is None:
            self.get_logger().warn('Keine Linienposition empfangen', throttle_duration_sec=2.0)
            self.publish_stop()
            return

        # 3. Liniendaten zu alt? (Timeout nach 0.5 Sekunden)
        if self.last_line_time is not None:
            age = (self.get_clock().now() - self.last_line_time).nanoseconds / 1e9
            if age > 0.5:
                self.get_logger().warn('Liniendaten veraltet - STOPP')
                self.publish_stop()
                return

        # ===== PID-Regelung =====

        # TODO: Fehler berechnen
        # Sollwert: 0 (Linie in Bildmitte)
        # Istwert: self.line_position
        # Fehler: Sollwert - Istwert
        error = 0.0  # TODO: 0.0 - self.line_position

        # TODO: PID-Regler aufrufen
        angular_velocity = 0.0  # TODO: self.pid.update(error)

        # Begrenzung der Winkelgeschwindigkeit
        angular_velocity = max(-self.max_angular_vel,
                              min(self.max_angular_vel, angular_velocity))

        # ===== Geschwindigkeit publizieren =====

        twist.linear.x = self.linear_vel
        twist.angular.z = angular_velocity

        self.cmd_vel_pub.publish(twist)

        # Debug-Ausgabe
        self.get_logger().debug(
            f'Line: {self.line_position:.2f}, Error: {error:.2f}, '
            f'Angular: {angular_velocity:.2f}'
        )

    def publish_stop(self):
        """Sendet Stopp-Befehl (alle Geschwindigkeiten = 0)."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Sicherer Stopp bei Ctrl+C
        node.publish_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
