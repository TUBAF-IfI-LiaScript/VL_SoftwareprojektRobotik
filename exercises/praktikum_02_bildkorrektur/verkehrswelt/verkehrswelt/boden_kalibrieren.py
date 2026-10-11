"""P 2: Homographie Bodenebene <- entzerrtes Kamerabild aus einem flach liegenden Schachbrett bestimmen.

Aufruf (Brett liegt mittig vor dem Roboter, Bild kommt von /camera/image_rect):
    ros2 run verkehrswelt boden_kalibrieren --ros-args -p abstand:=0.185
Schreibt die Parameter für image_corrector nach homographie.yaml.

Vorlage: Ergänzen Sie TODO 4 und TODO 5.
Autoren: Sebastian Zug & Claude Code (Opus 5.5) mit LiaScript Teaching-Agent
"""
import cv2
import numpy as np
import rclpy
import rclpy.executors
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from verkehrswelt.kamera import boden_nach_pixel


class BodenKalibrieren(Node):
    def __init__(self):
        super().__init__('boden_kalibrieren')
        self.declare_parameter('ecken_x', 8)          # innere Ecken quer zum Roboter
        self.declare_parameter('ecken_y', 6)          # innere Ecken in Fahrtrichtung
        self.declare_parameter('feld', 0.025)         # Kantenlänge eines Feldes in m
        self.declare_parameter('abstand', 0.20)       # Radachse -> nächste Eckenreihe in m (gemessen)
        self.declare_parameter('px_pro_m', 400.0)
        self.declare_parameter('x_min', 0.12)
        self.declare_parameter('x_max', 0.62)
        self.declare_parameter('y_max', 0.35)
        self.declare_parameter('datei', 'homographie.yaml')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_rect', self.bild, 10)
        self.fertig = False

    def bild(self, msg):
        if self.fertig:
            return
        p = {n: self.get_parameter(n).value for n in
             ['ecken_x', 'ecken_y', 'feld', 'abstand', 'px_pro_m', 'x_min', 'x_max', 'y_max', 'datei']}
        grau = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2GRAY)
        ok, ecken = cv2.findChessboardCorners(grau, (p['ecken_x'], p['ecken_y']))
        if not ok:
            self.get_logger().warn('Schachbrett nicht gefunden – liegt es vollständig im Bild?', throttle_duration_sec=2.0)
            return
        ecken = cv2.cornerSubPix(grau, ecken, (5, 5), (-1, -1),
                                 (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)).reshape(-1, 2)
        # findChessboardCorners nummeriert von einer beliebigen Bildecke aus: Reihenfolge an der Lage im Bild festmachen
        ecken = sortiere(ecken, p['ecken_x'], p['ecken_y'])
        n = len(ecken)
        # TODO 4 (VL 4): Bodenkoordinaten (x, y) jeder Ecke in base_footprint aufstellen – in derselben
        #   Reihenfolge wie 'ecken' (eine Zeile darüber sortiert): erst Reihe j = 0 (am nächsten am Roboter,
        #   bei x = abstand), darin Spalte i = 0 .. ecken_x-1 von RECHTS nach LINKS; dann Reihe j = 1 usw.
        #   x wächst nach vorn, y nach links; die Brettmitte liegt auf y = 0, rechts ist y also negativ.
        #   Beispiel: die erste Ecke (i = 0, j = 0) liegt bei (abstand, -(ecken_x - 1) / 2 * feld).
        boden = np.zeros((n, 2))
        ziel = np.array([boden_nach_pixel(x, y, p['x_max'], p['y_max'], p['px_pro_m']) for x, y in boden])
        # TODO 5 (VL 7): Homographie bestimmen, die Bildpunkte 'ecken' auf die Zielpixel 'ziel' abbildet.
        H = np.eye(3)
        if not lage_plausibel(H, ecken):
            self.get_logger().error('Die Draufsicht wäre gespiegelt oder verdreht: rechts im Bild muss rechts '
                                    '(y < 0), oben im Bild muss weiter vorn (größeres x) liegen – TODO 4 prüfen')
            self.fertig = True
            return
        fehler = np.linalg.norm(cv2.perspectiveTransform(ecken.reshape(-1, 1, 2), H).reshape(-1, 2) - ziel, axis=1)
        param = {'image_corrector': {'ros__parameters': {
            'homographie': [float(v) for v in (H / H[2, 2]).ravel()],
            'px_pro_m': p['px_pro_m'], 'x_min': p['x_min'], 'x_max': p['x_max'], 'y_max': p['y_max']}}}
        with open(p['datei'], 'w') as f:
            yaml.safe_dump(param, f, sort_keys=False)
        self.get_logger().info(f'{n} Ecken, mittlerer Fehler {fehler.mean() / p["px_pro_m"] * 1000:.1f} mm '
                               f'-> {p["datei"]} geschrieben')
        self.fertig = True


def lage_plausibel(H, ecken):
    """Wird ein Bildpunkt rechts von der Brettmitte auch in der Draufsicht rechts abgebildet,
    und ein Punkt weiter oben im Bild auch weiter vorn (= weiter oben in der Draufsicht)?"""
    mitte = ecken.mean(axis=0)
    probe = np.array([mitte, mitte + [20, 0], mitte + [0, -20]], dtype=np.float64).reshape(-1, 1, 2)
    m, r, o = cv2.perspectiveTransform(probe, H).reshape(-1, 2)
    return r[0] > m[0] and o[1] < m[1]


def sortiere(ecken, nx, ny):
    """Ecken so ordnen, dass Reihe 0 dem Roboter am nächsten liegt (unten im Bild)
    und innerhalb einer Reihe von rechts nach links (y wächst nach links)."""
    e = ecken.reshape(ny, nx, 2)
    # erst Reihen: die Reihe mit größter Bildzeile (unten) ist die nächste
    if e[0, :, 1].mean() < e[-1, :, 1].mean():
        e = e[::-1]
    # dann Spalten: in Reihe 0 soll die Spalte mit größter Bildspalte (rechts) zuerst kommen
    if e[0, 0, 0] < e[0, -1, 0]:
        e = e[:, ::-1]
    return e.reshape(-1, 2)


def main():
    rclpy.init()
    knoten = BodenKalibrieren()
    try:
        while rclpy.ok() and not knoten.fertig:
            rclpy.spin_once(knoten, timeout_sec=0.5)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
