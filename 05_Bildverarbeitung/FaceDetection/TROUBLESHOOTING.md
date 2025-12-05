# Troubleshooting - Face Detection

## Problem gelöst: "No image on /face_detection/image"

### Symptome
- `ros2 launch face_detector face_detection.launch.py` läuft
- Kamera publiziert `/image_raw`
- Face Detector erkennt Gesichter (Logs zeigen "X Gesicht(er) erkannt")
- **ABER**: `/face_detection/image` zeigt kein Bild in rqt_image_view

### Ursachen & Lösungen

#### 1. OpenCV Haar Cascade nicht gefunden ✅ GELÖST

**Problem**: `cv2.data.haarcascades` existiert nicht in älteren OpenCV-Versionen (< 4.7)

**Fehlermeldung**:
```
AttributeError: module 'cv2' has no attribute 'data'
```

**Lösung**: Automatische Pfad-Suche implementiert:
- `/usr/share/opencv4/haarcascades/` (System-Installation)
- `/usr/share/opencv/haarcascades/` (Fallback)
- Python-Paket-Pfad (falls vorhanden)

**Code-Fix**: Siehe `face_detector_node.py` Zeilen 51-75

#### 2. QoS-Mismatch zwischen v4l2_camera und face_detector ✅ GELÖST

**Problem**: v4l2_camera nutzt `BEST_EFFORT`, face_detector nutzte `RELIABLE`

**Symptom**:
```bash
ros2 topic list  # Zeigt NICHT /image_raw oder /face_detection/image
ros2 topic hz /image_raw  # Funktioniert trotzdem (zeigt ~19 Hz)
```

**Lösung**: QoS-Profile angepasst:

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # ← Wichtig!
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

self.image_sub = self.create_subscription(
    Image,
    '/image_raw',
    self.image_callback,
    sensor_qos  # ← Verwende sensor_qos statt '10'
)
```

**Code-Fix**: Siehe `face_detector_node.py` Zeilen 85-119

---

## Verification Steps

### Nach dem Fix überprüfen:

```bash
# 1. Node manuell starten
source install/setup.bash
ros2 run face_detector face_detector_node

# Erwartete Ausgabe:
# [INFO] [...] Nutze Haar Cascade: /usr/share/opencv4/haarcascades/...
# [INFO] [...] Haar Cascade erfolgreich geladen
# [INFO] [...] Face Detector Node gestartet
# [INFO] [...] X Gesicht(er) erkannt
```

```bash
# 2. Topics prüfen
ros2 topic list

# Sollte zeigen:
# /face_detection/image
# /face_detection/faces
# /image_raw
# /parameter_events
# /rosout
```

```bash
# 3. QoS-Profile prüfen
ros2 topic info /image_raw --verbose
ros2 topic info /face_detection/image --verbose

# Beide sollten kompatible QoS haben:
# Reliability: BEST_EFFORT (Subscriber) kompatibel mit BEST_EFFORT (Publisher)
```

```bash
# 4. Bild-Daten prüfen
ros2 topic hz /face_detection/image

# Sollte ~7-20 Hz zeigen
```

```bash
# 5. Viewer testen
ros2 run rqt_image_view rqt_image_view /face_detection/image

# Sollte Kamerabild mit grünen Rechtecken um Gesichter zeigen
```

---

## Weitere häufige Probleme

### Problem: "No module named 'cv2'"

**Lösung**:
```bash
pip3 install opencv-python
# Oder system-weit:
sudo apt install python3-opencv
```

### Problem: "Unable to import 'cv_bridge'"

**Lösung**:
```bash
sudo apt install ros-jazzy-cv-bridge
```

### Problem: Kamera nicht gefunden

**Symptom**:
```
[ERROR] Failed to open camera /dev/video0
```

**Lösung**:
```bash
# Verfügbare Kameras auflisten
v4l2-ctl --list-devices

# Berechtigungen prüfen
ls -la /dev/video*

# User zur video-Gruppe hinzufügen
sudo usermod -a -G video $USER
# Dann neu einloggen!

# Andere Kamera nutzen
ros2 launch face_detector face_detection.launch.py video_device:=/dev/video2
```

### Problem: Launch-File findet Paket nicht

**Symptom**:
```
Package 'face_detector' not found
```

**Lösung**:
```bash
# Workspace neu bauen
cd FaceDetection
colcon build --symlink-install

# Environment sourcen
source install/setup.bash

# Paket prüfen
ros2 pkg list | grep face_detector
```

### Problem: Langsame Performance / niedrige FPS

**Symptom**: FPS < 5, Verzögerung sichtbar

**Lösungen**:

1. **Auflösung reduzieren**:
```bash
ros2 launch face_detector face_detection.launch.py \
    image_width:=320 image_height:=240
```

2. **Parameter optimieren**:
```bash
ros2 run face_detector face_detector_node --ros-args \
    -p scale_factor:=1.3 \
    -p min_size_width:=60 \
    -p min_size_height:=60
```

3. **CPU-Profiling** (optional):
```bash
# Installiere py-spy
pip3 install py-spy

# Profile den Node
sudo py-spy top --pid $(pgrep -f face_detector_node)
```

---

## Debug-Kommandos

```bash
# 1. Alle laufenden ROS2-Nodes
ros2 node list

# 2. Info über einen Node
ros2 node info /face_detector_node

# 3. Alle Topics mit Publishers/Subscribers
ros2 topic list -v

# 4. Live-Log eines Nodes
ros2 run face_detector face_detector_node --ros-args --log-level DEBUG

# 5. rqt_graph visualisieren
rqt_graph

# 6. Topic-Echo (zeigt rohe Daten)
ros2 topic echo /face_detection/faces

# 7. Parameter eines laufenden Nodes ändern
ros2 param set /face_detector_node scale_factor 1.2
```

---

## Bekannte Limitationen

1. **Nur frontale Gesichter**: Haar Cascade erkennt nur frontal ausgerichtete Gesichter
2. **Beleuchtungsabhängig**: Schlechtes Licht → schlechte Erkennung
3. **CPU-basiert**: Kein GPU-Support, daher langsam bei hoher Auflösung
4. **False Positives**: Bei komplexen Hintergründen möglich

**Alternativen für bessere Performance**:
- YOLO (You Only Look Once) für GPU-beschleunigte Detektion
- MediaPipe Face Detection (Google)
- DNN-basierte Modelle in OpenCV

---

## Changelog

**Version 0.1.1** (2024-12-05)
- ✅ Fix: OpenCV Haar Cascade Pfad-Suche für ältere Versionen
- ✅ Fix: QoS-Profile für v4l2_camera-Kompatibilität
- ✅ Fix: Entfernt ungenutztes numpy-Import

**Version 0.1.0** (2024-12-05)
- Initial Release

---

**Bei weiteren Problemen**: sebastian.zug@informatik.tu-freiberg.de
