# Quick Start - Face Detection

Schnellanleitung für den Einstieg in 5 Minuten.

## 1. Build (einmalig)

```bash
cd ~/Desktop/Vorlesungen/WiSe_2025_26/VL_SoftwareprojektRobotik/05_Bildverarbeitung/FaceDetection
./build.sh
```

Oder manuell:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## 2. Workspace aktivieren

```bash
# Wichtig: Immer vom Workspace-Root aus sourcen!
cd ~/Desktop/Vorlesungen/WiSe_2025_26/VL_SoftwareprojektRobotik/05_Bildverarbeitung/FaceDetection
source install/setup.bash

# Oder mit absolutem Pfad:
source ~/Desktop/Vorlesungen/WiSe_2025_26/VL_SoftwareprojektRobotik/05_Bildverarbeitung/FaceDetection/install/setup.bash
```

**Hinweis**: Bei Fehlern wie "no such file or directory: local_setup.bash":
```bash
# Clean rebuild:
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

## 3. Kamera testen

```bash
# Verfügbare Kameras anzeigen
./test_camera.sh

# Oder direkt:
v4l2-ctl --list-devices
```

## 4. System starten

### Option A: Alles mit einem Befehl (empfohlen)

```bash
ros2 launch face_detector face_detection.launch.py
```

Das startet:
- ✅ Kamera (v4l2_camera)
- ✅ Face Detector
- ✅ Image Viewer (rqt_image_view)
- ✅ Bag-Recording

### Option B: Ohne Recording

```bash
ros2 launch face_detector face_detection.launch.py record_bag:=false
```

### Option C: Mit anderer Kamera

```bash
ros2 launch face_detector face_detection.launch.py video_device:=/dev/video2
```

## 5. Manuell testen (Schritt-für-Schritt)

### Terminal 1: Kamera

```bash
source install/setup.bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
```

### Terminal 2: Face Detector

```bash
source install/setup.bash
ros2 run face_detector face_detector_node
```

### Terminal 3: Viewer

```bash
source install/setup.bash
ros2 run rqt_image_view rqt_image_view /face_detection/image
```

## 6. Topics inspizieren

```bash
# Alle Topics anzeigen
ros2 topic list

# Relevante Topics:
# /image_raw - Rohe Kamerabilder
# /face_detection/image - Annotierte Bilder
# /face_detection/faces - Anzahl Gesichter

# Topic-Info anzeigen
ros2 topic info /face_detection/faces

# Anzahl Gesichter live sehen
ros2 topic echo /face_detection/faces
```

## 7. Ergebnis prüfen

Im rqt_image_view sollten Sie:
- ✅ Live-Kamerabild sehen
- ✅ Grüne Rechtecke um erkannte Gesichter
- ✅ "Faces: X" Text oben links

## Häufige Probleme

### Kamera nicht gefunden

```bash
# Prüfen
ls -la /dev/video*

# Berechtigungen
sudo usermod -a -G video $USER
# Neu einloggen!
```

### Node startet nicht

```bash
# ROS2 Environment sourcen
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Dependencies prüfen
sudo apt install ros-jazzy-v4l2-camera ros-jazzy-cv-bridge
pip3 install opencv-python
```

### Keine Gesichter erkannt

- Ausreichend Licht?
- Gesicht frontal zur Kamera?
- Parameter anpassen:

```bash
ros2 run face_detector face_detector_node --ros-args \
    -p scale_factor:=1.05 \
    -p min_neighbors:=3
```

## Nächste Schritte

📖 Siehe [README.md](README.md) für:
- Detaillierte Parameter-Beschreibungen
- Erweiterte Konfiguration
- Performance-Tuning
- Fehlerbehebung

---

**Viel Erfolg!** 🤖
