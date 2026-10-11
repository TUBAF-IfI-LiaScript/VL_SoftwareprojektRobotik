"""P 2: Kamerabild entzerren und in die Vogelperspektive auf die Fahrbahn umrechnen.

Vorlage: Ergänzen Sie die Stellen TODO 1 bis TODO 3. Alles andere ist fertig.
Autoren: Sebastian Zug & Claude Code (Opus 5.5) mit LiaScript Teaching-Agent
"""
import cv2
import numpy as np
import rclpy
import rclpy.executors
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from verkehrswelt.kamera import lade_kalibrierung


class ImageCorrector(Node):
    def __init__(self):
        super().__init__('image_corrector')
        self.declare_parameter('kalibrierdatei', '')
        self.declare_parameter('komprimiert', True)    # live: JPEG über WLAN, Bag aus P 1: false
        self.declare_parameter('homographie', [0.0] * 9)
        self.declare_parameter('px_pro_m', 400.0)
        self.declare_parameter('x_min', 0.12)
        self.declare_parameter('x_max', 0.62)
        self.declare_parameter('y_max', 0.35)
        self.declare_parameter('raster', True)          # zusätzlich Bild mit 10-cm-Raster zum Ablesen

        datei = self.get_parameter('kalibrierdatei').value
        try:
            K, D, (w, h) = lade_kalibrierung(datei)
        except OSError:
            self.get_logger().fatal(f'Kalibrierdatei nicht gefunden: "{datei}" – Parameter kalibrierdatei prüfen (Teil A)')
            raise SystemExit(1)
        self.groesse = (w, h)
        # TODO 1 (Ü 4, VL 7): Entzerrungstabellen einmal berechnen, damit jedes Bild nur noch nachgeschlagen wird.
        #   cv2.initUndistortRectifyMap(K, D, None, K, (w, h), cv2.CV_16SC2) liefert zwei Tabellen.
        self.map1, self.map2 = None, None
        self.H = np.array(self.get_parameter('homographie').value, dtype=np.float64).reshape(3, 3)
        s = self.get_parameter('px_pro_m').value
        x_min, x_max = self.get_parameter('x_min').value, self.get_parameter('x_max').value
        y_max = self.get_parameter('y_max').value
        self.groesse_vogel = (int(round(2 * y_max * s)), int(round((x_max - x_min) * s)))
        self.mit_vogel = abs(self.H[2, 2]) > 0

        self.bridge = CvBridge()
        if self.get_parameter('komprimiert').value:
            self.sub = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.bild, qos_profile_sensor_data)
        else:
            self.sub = self.create_subscription(Image, '/camera/image_raw', self.bild, qos_profile_sensor_data)
        self.pub_rect = self.create_publisher(Image, '/camera/image_rect', 10)
        self.pub_vogel = self.create_publisher(Image, '/camera/image_birdseye', 10)
        self.pub_raster = self.create_publisher(Image, '/camera/image_birdseye_raster', 10)
        self.px_pro_m, self.x_max, self.y_max = s, x_max, y_max
        self.get_logger().info(f'Kalibrierung geladen: fx={K[0, 0]:.1f}, k1={D[0]:.3f}; '
                               f'Vogelperspektive {"aktiv" if self.mit_vogel else "noch ohne Homographie"}')

    def bild(self, msg):
        if isinstance(msg, CompressedImage):
            bild = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        else:
            bild = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # TODO 2 (Ü 4): Bild mit den Tabellen aus TODO 1 entzerren (cv2.remap, Interpolation INTER_LINEAR).
        if (bild.shape[1], bild.shape[0]) != self.groesse:
            self.get_logger().error(f'Bild {bild.shape[1]}x{bild.shape[0]} passt nicht zur Kalibrierung '
                                    f'{self.groesse[0]}x{self.groesse[1]} – andere Kamera oder Auflösung?',
                                    throttle_duration_sec=5.0)
            return
        entzerrt = bild
        aus = self.bridge.cv2_to_imgmsg(entzerrt, 'bgr8')
        aus.header = msg.header
        self.pub_rect.publish(aus)
        if self.mit_vogel:
            # TODO 3 (VL 7): entzerrtes Bild mit der Homographie self.H in die Vogelperspektive abbilden.
            #   Zielgröße (Breite, Höhe) steht in self.groesse_vogel.
            vogel = entzerrt
            aus = self.bridge.cv2_to_imgmsg(vogel, 'bgr8')
            aus.header.stamp = msg.header.stamp
            aus.header.frame_id = 'base_footprint'
            self.pub_vogel.publish(aus)
            if self.get_parameter('raster').value:
                self.pub_raster.publish(self.bridge.cv2_to_imgmsg(self.zeichne_raster(vogel), 'bgr8'))

    def zeichne_raster(self, vogel):
        """Linien alle 10 cm, die Linie y = 0 (Mitte des Roboters) rot."""
        bild = vogel.copy()
        s, h, w = self.px_pro_m, bild.shape[0], bild.shape[1]
        for k in range(-10, 11):
            spalte = int(round((self.y_max - k * 0.1) * s))
            if 0 <= spalte < w:
                cv2.line(bild, (spalte, 0), (spalte, h), (0, 0, 255) if k == 0 else (0, 160, 0), 1)
            zeile = int(round((self.x_max - k * 0.1) * s))
            if 0 <= zeile < h:
                cv2.line(bild, (0, zeile), (w, zeile), (0, 160, 0), 1)
        return bild


def main():
    rclpy.init()
    try:
        rclpy.spin(ImageCorrector())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
