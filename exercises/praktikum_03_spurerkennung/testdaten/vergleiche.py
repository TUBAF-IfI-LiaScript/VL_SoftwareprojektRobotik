"""Vergleicht /lane/offset mit der Sollkurve der Testfahrt (offset_soll.csv).

Aufruf, während Ihre Knoten laufen und BEVOR Sie die Testaufnahme abspielen:
    python3 vergleiche.py offset_soll.csv
Beenden mit Strg+C, sobald die Aufnahme durchgelaufen ist.

Autoren: Sebastian Zug & Claude Code (Opus 5.5) mit LiaScript Teaching-Agent
"""
import sys
import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32

T0 = 1_768_000_000  # Startzeit der Testaufnahme in s


class Vergleich(Node):
    def __init__(self, soll):
        super().__init__('vergleiche')
        self.soll = soll
        self.letztes_bild = None
        self.paare = []
        self.bilder = 0
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.bild, qos_profile_sensor_data)
        self.create_subscription(Float32, '/lane/offset', self.offset, 10)

    def bild(self, msg):
        self.letztes_bild = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.bilder += 1

    def offset(self, msg):
        # Ein Wert gehört zum zuletzt gesendeten Bild (Ihre Knoten brauchen nur wenige ms)
        if self.letztes_bild is not None:
            k = int(round((self.letztes_bild - T0) / 0.1))
            if 0 <= k < len(self.soll):
                self.paare.append((self.soll[k], msg.data))


def main():
    soll = np.loadtxt(sys.argv[1], delimiter=';', skiprows=1)[:, 1]
    rclpy.init()
    knoten = Vergleich(soll)
    try:
        rclpy.spin(knoten)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    if not knoten.paare:
        print('Keine Werte auf /lane/offset empfangen.')
        return
    s, i = np.array(knoten.paare).T
    f = i - s
    deutlich = np.abs(s) > 0.01
    abdeckung = len(f) / max(knoten.bilder, 1) * 100
    vorzeichen = np.mean(np.sign(i[deutlich]) == np.sign(s[deutlich])) * 100
    pruefung = [
        ('Werte erhalten', f'{len(f)} zu {knoten.bilder} empfangenen Bildern ({abdeckung:.0f} %)', abdeckung >= 90),
        ('systematischer Fehler |Mittelwert|', f'{abs(np.mean(f)) * 1000:.1f} mm', abs(np.mean(f)) < 0.005),
        ('größte Abweichung', f'{np.max(np.abs(f)) * 1000:.1f} mm', np.max(np.abs(f)) < 0.03),
        ('Vorzeichen richtig (|Soll| > 1 cm)', f'{vorzeichen:.1f} %', vorzeichen > 95),
    ]
    for name, wert, ok in pruefung:
        print(f'{"ok    " if ok else "NICHT "} {name:36s} {wert}')
    print('Selbsttest bestanden.' if all(p[2] for p in pruefung) else 'Selbsttest noch nicht bestanden.')


if __name__ == '__main__':
    main()
