"""P 3: Eigene Fahrspur in der Vogelperspektive erkennen.

Veröffentlicht auf /lane/offset (std_msgs/Float32) die seitliche Abweichung des Roboters von der
Mitte der rechten Fahrspur in m, gemessen an der Stelle 'vorausschau' vor base_footprint:
positiv = Roboter steht links von der Spurmitte. Ist keine Linie zu sehen, wird nichts veröffentlicht.

Vorlage: Kopieren Sie diese Datei nach verkehrswelt/verkehrswelt/ und ergänzen Sie TODO 1 bis TODO 5.
Die Vorlage ist lauffähig, liefert aber noch keine sinnvollen Werte.
Autoren: Sebastian Zug & Claude Code (Opus 5.5) mit LiaScript Teaching-Agent
"""
from itertools import combinations

import cv2
import numpy as np
import rclpy
import rclpy.executors
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from verkehrswelt.kamera import pixel_nach_boden


class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        for name, wert in [('schwelle', 110), ('min_flaeche', 40), ('strich_min_laenge', 0.05),
                           ('strich_max_laenge', 0.16), ('spurbreite', 0.22), ('vorausschau', 0.30),
                           ('px_pro_m', 400.0), ('x_max', 0.62), ('y_max', 0.35)]:
            self.declare_parameter(name, wert)
        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_birdseye', self.bild, 10)
        self.pub = self.create_publisher(Float32, '/lane/offset', 10)
        self.pub_debug = self.create_publisher(Image, '/lane/debug', 10)

    def p(self, name):
        return self.get_parameter(name).value

    def bild(self, msg):
        s, x_max, y_max = self.p('px_pro_m'), self.p('x_max'), self.p('y_max')
        grau = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2GRAY)

        # 1) weiße Markierungen freistellen und säubern (VL 6)
        # TODO 1 (VL 6): Binärmaske – Pixel heller als self.p('schwelle') werden 255, alle anderen 0.
        #  Achtung: cv2.threshold liefert ZWEI Rückgabewerte (Schwelle, Bild).
        maske = np.zeros_like(grau)
        # TODO 2 (VL 6): einzelne Störpixel mit einem Opening (3x3-Kern) entfernen.

        # 2) zusammenhängende Flächen nach ihrer Form einteilen (VL 6)
        n, label, stats, _ = cv2.connectedComponentsWithStats(maske)
        striche, raender = [], []
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] < self.p('min_flaeche'):
                continue
            zeilen, spalten = np.nonzero(label == i)
            x, y = pixel_nach_boden(spalten, zeilen, x_max, y_max, s)
            laenge = np.hypot(np.ptp(x), np.ptp(y))
            if laenge < self.p('strich_min_laenge'):
                continue  # Stummel, z.B. am Bildrand abgeschnitten – zu unsicher
            # TODO 3 (VL 6): kürzer als self.p('strich_max_laenge') -> Strich der Mittellinie, sonst
            #  Stück einer Randlinie. Hängen Sie (x, y, i) an die passende Liste an.

        # 3) Linien an der Stelle x = vorausschau auswerten (y-Koordinate in base_footprint)
        xv = self.p('vorausschau')
        striche = mittellinie_auswaehlen(striche)
        mitte = None
        if len(striche) >= 2:
            x = np.concatenate([c[0] for c in striche]); y = np.concatenate([c[1] for c in striche])
            mitte = np.polyval(np.polyfit(x, y, 1), xv)
        # rechte Randlinie: lange Linie rechts der Mittellinie (ohne Mittellinie: rechts vom Roboter)
        grenze = mitte if mitte is not None else 0.05
        rechts_davon = [c for c in raender if np.median(c[1]) < grenze and np.ptp(c[0]) > 0.05]
        rechts = None
        if rechts_davon:
            x, y, _ = max(rechts_davon, key=lambda c: np.median(c[1]))
            rechts = np.polyval(np.polyfit(x, y, 2 if np.ptp(x) > 0.2 else 1), xv)
        b = self.p('spurbreite')
        if mitte is not None and rechts is not None and abs(mitte - rechts - b) > 0.06:
            mitte = None  # Abstand passt nicht zur Spurbreite: der langen Randlinie mehr trauen

        # TODO 4: y der Spurmitte bestimmen (Rechtsverkehr: eigene Spur zwischen Randlinie 'rechts' und
        #  Mittellinie 'mitte'). Fehlt eine der beiden (None), aus der anderen und der Spurbreite b
        #  schätzen. Fehlen beide, bleibt spurmitte = None.
        spurmitte = None

        dbg = self.debugbild(grau, label, striche, raender, spurmitte)
        if spurmitte is None:
            self.get_logger().warn('Keine Fahrbahnlinie erkannt – kein Wert', throttle_duration_sec=1.0)
        else:
            # TODO 5 (VL 4): Abweichung des Roboters von der Spurmitte – positiv, wenn der Roboter LINKS
            #  der Spurmitte steht. Der Roboter liegt bei y = 0. (spurmitte selbst nicht verändern,
            #  sie wird oben für den roten Punkt im Debug-Bild gebraucht.)
            abweichung = 0.0
            self.pub.publish(Float32(data=float(abweichung)))
        aus = self.bridge.cv2_to_imgmsg(dbg, 'bgr8')
        aus.header = msg.header
        self.pub_debug.publish(aus)

    def debugbild(self, grau, label, striche, raender, spurmitte):
        """Striche gelb, Randlinien blau, Mittelachse des Roboters dunkelrot, Spurmitte als roter Punkt."""
        s, x_max, y_max = self.p('px_pro_m'), self.p('x_max'), self.p('y_max')
        dbg = cv2.cvtColor(grau, cv2.COLOR_GRAY2BGR)
        for _, _, i in striche:
            dbg[label == i] = (0, 220, 255)
        for _, _, i in raender:
            dbg[label == i] = (255, 120, 0)
        cv2.line(dbg, (int(y_max * s), 0), (int(y_max * s), dbg.shape[0]), (0, 0, 180), 1)
        zeile = int((x_max - self.p('vorausschau')) * s)
        if spurmitte is None:
            cv2.putText(dbg, 'keine Linie', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        else:
            cv2.circle(dbg, (int((y_max - spurmitte) * s), zeile), 6, (0, 0, 255), -1)
        return dbg


def mittellinie_auswaehlen(striche, grenze=0.04):
    """Welche Stücke gehören zur Mittellinie? Für jedes Paar von Stücken eine Gerade legen und zählen,
    wie viele Stücke höchstens 'grenze' (m) daneben liegen. Es gewinnt die Gerade mit den meisten
    Stücken, bei Gleichstand die, die am geradesten in Fahrtrichtung verläuft (Idee von RANSAC).
    So fallen Bruchstücke anderer Linien heraus, etwa das Ende einer Randlinie an einer Einmündung."""
    if len(striche) < 2:
        return list(striche)
    beste, schluessel = [], None
    for a, b in combinations(striche, 2):
        g = np.polyfit(np.concatenate([a[0], b[0]]), np.concatenate([a[1], b[1]]), 1)
        if abs(g[0]) > 0.5:
            continue  # steiler als ~27° zur Fahrtrichtung: keine Mittellinie
        dabei = [c for c in striche if abs(np.median(c[1] - np.polyval(g, c[0]))) <= grenze]
        k = (len(dabei), -abs(g[0]))
        if schluessel is None or k > schluessel:
            beste, schluessel = dabei, k
    return beste


def main():
    rclpy.init()
    try:
        rclpy.spin(LaneDetector())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
