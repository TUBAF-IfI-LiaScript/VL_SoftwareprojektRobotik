# Face Detection ROS2 Workspace

ROS2-Workspace für Echtzeit-Gesichtserkennung mit OpenCV Haar Cascade.

## Überblick

Dieser Workspace enthält:

- **face_detector_node**: Python-Node für Gesichtserkennung
- **face_detection.launch.py**: Launch-File für komplettes System
- **v4l2_camera**: Kamera-Treiber für USB-Kameras
- **rqt_image_view**: Visualisierung der Ergebnisse
- **rosbag2**: Automatische Datenaufzeichnung

## System-Architektur

```
┌─────────────┐      /image_raw       ┌──────────────────┐
│ v4l2_camera │ ──────────────────────>│ face_detector    │
│   (Kamera)  │                        │      _node       │
└─────────────┘                        └──────────────────┘
                                              │
                                              │ /face_detection/image
                                              │ /face_detection/faces
                                              v
                                       ┌──────────────────┐
                                       │ rqt_image_view   │
                                       │   (Viewer)       │
                                       └──────────────────┘
                                              │
                                              v
                                       ┌──────────────────┐
                                       │   rosbag2        │
                                       │  (Aufzeichnung)  │
                                       └──────────────────┘
```

## Installation

### 1. Abhängigkeiten installieren

```bash
# ROS2 Jazzy Pakete
sudo apt update
sudo apt install ros-jazzy-v4l2-camera \
                 ros-jazzy-rqt-image-view \
                 ros-jazzy-cv-bridge \
                 ros-jazzy-rosbag2-storage-mcap \
                 v4l-utils

# Python-Pakete
pip3 install opencv-python
```

### 2. Workspace bauen

```bash
cd ~/Desktop/Vorlesungen/WiSe_2025_26/VL_SoftwareprojektRobotik/05_Bildverarbeitung/FaceDetection
colcon build
source install/setup.bash
```

## Verwendung

### Schnellstart: Komplettes System starten

```bash
# Source workspace
source install/setup.bash

# Starte alles (Kamera, Detektor, Viewer, Recording)
ros2 launch face_detector face_detection.launch.py
```

### Verfügbare Kameras anzeigen

```bash
v4l2-ctl --list-devices
```

### Mit spezifischer Kamera starten

```bash
ros2 launch face_detector face_detection.launch.py video_device:=/dev/video2
```

### Ohne Aufzeichnung starten

```bash
ros2 launch face_detector face_detection.launch.py record_bag:=false
```

### Ohne Viewer (Headless)

```bash
ros2 launch face_detector face_detection.launch.py show_image:=false
```

### Alle Parameter anpassen

```bash
ros2 launch face_detector face_detection.launch.py \
    video_device:=/dev/video0 \
    image_width:=1280 \
    image_height:=720 \
    record_bag:=true \
    show_image:=true
```

## Manuelle Verwendung (ohne Launch-File)

### Terminal 1: Kamera starten

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
```

### Terminal 2: Face Detector starten

```bash
ros2 run face_detector face_detector_node
```

### Terminal 3: Bild anzeigen

```bash
ros2 run rqt_image_view rqt_image_view /face_detection/image
```

### Terminal 4 (Optional): Daten aufzeichnen

```bash
ros2 bag record -o face_detection /image_raw /face_detection/image /face_detection/faces
```

## Topics

| Topic | Message Type | Beschreibung |
|-------|--------------|--------------|
| `/image_raw` | `sensor_msgs/Image` | Rohe Kamerabilder (Input) |
| `/face_detection/image` | `sensor_msgs/Image` | Annotierte Bilder mit Gesichts-Rechtecken |
| `/face_detection/faces` | `std_msgs/Int32` | Anzahl erkannter Gesichter |

## Parameter

### face_detector_node Parameter

```bash
ros2 run face_detector face_detector_node --ros-args \
    -p scale_factor:=1.1 \
    -p min_neighbors:=5 \
    -p min_size_width:=30 \
    -p min_size_height:=30
```

| Parameter | Default | Beschreibung |
|-----------|---------|--------------|
| `scale_factor` | 1.1 | Skalierungsfaktor für Bildpyramide (1.1 - 2.0) |
| `min_neighbors` | 5 | Mindestanzahl Nachbarn für valide Detektion (3-10) |
| `min_size_width` | 30 | Minimale Gesichtsbreite in Pixel |
| `min_size_height` | 30 | Minimale Gesichtshöhe in Pixel |
| `cascade_file` | '' | Pfad zu eigenem Haar Cascade (optional) |

**Parameter-Tuning**:
- **Mehr Detektionen** (höhere Sensitivität): `scale_factor=1.05`, `min_neighbors=3`
- **Weniger False Positives**: `scale_factor=1.2`, `min_neighbors=7`
- **Schnellere Verarbeitung**: `scale_factor=1.3`, größere `min_size`

## Fehlerbehebung

### Problem: "No video device found"

```bash
# Liste verfügbare Kameras
v4l2-ctl --list-devices

# Teste Kamera-Zugriff
v4l2-ctl --device=/dev/video0 --all

# Berechtigungen prüfen
ls -la /dev/video*
sudo usermod -a -G video $USER
# Dann neu einloggen!
```

### Problem: "Haar Cascade nicht gefunden"

Der Standard-Cascade wird automatisch geladen. Falls manuell angegeben:

```bash
# Prüfe ob OpenCV Cascades installiert sind
python3 -c "import cv2; print(cv2.data.haarcascades)"

# Falls nicht, installiere opencv-python neu
pip3 install --upgrade opencv-python
```

### Problem: "cv_bridge ImportError"

```bash
# ROS2 cv_bridge neu installieren
sudo apt install ros-jazzy-cv-bridge

# Python-OpenCV Version prüfen (sollte kompatibel sein)
pip3 list | grep opencv
```

### Problem: Schlechte Detektions-Performance

**Zu viele False Positives**:
- Erhöhe `min_neighbors` (z.B. 7-8)
- Erhöhe `scale_factor` (z.B. 1.2)
- Erhöhe `min_size_width/height`

**Zu wenige Detektionen**:
- Reduziere `min_neighbors` (z.B. 3-4)
- Reduziere `scale_factor` (z.B. 1.05)
- Verbessere Beleuchtung

**Langsame Verarbeitung**:
- Reduziere Kamera-Auflösung (z.B. 320×240)
- Erhöhe `scale_factor`
- Erhöhe `min_size`

## ROS2-Bag-Dateien analysieren

```bash
# Bag-Info anzeigen
ros2 bag info face_detection_TIMESTAMP

# Bag abspielen (ohne Kamera)
ros2 bag play face_detection_TIMESTAMP

# In neuem Terminal: Detektor und Viewer starten
ros2 run face_detector face_detector_node
ros2 run rqt_image_view rqt_image_view /face_detection/image
```

## Erweiterte Nutzung

### Eigener Haar Cascade verwenden

```bash
# Lade z.B. Profil-Gesichter-Cascade
wget https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_profileface.xml

# Verwende eigenen Cascade
ros2 run face_detector face_detector_node --ros-args \
    -p cascade_file:=/path/to/haarcascade_profileface.xml
```

### Topic-Remapping

```bash
# Verwende andere Kamera-Topics
ros2 run face_detector face_detector_node --ros-args \
    --remap /image_raw:=/camera/color/image_raw
```

### Mehrere Kameras parallel

Siehe [examples/multi_camera.launch.py](examples/multi_camera.launch.py)

## Performance-Benchmarks

Typische Performance auf Standard-Hardware:

| Auflösung | FPS (Intel i5) | FPS (Raspberry Pi 4) |
|-----------|----------------|----------------------|
| 320×240   | ~30 fps        | ~15 fps              |
| 640×480   | ~20 fps        | ~8 fps               |
| 1280×720  | ~10 fps        | ~3 fps               |

*Mit Standard-Parametern, ohne GPU-Beschleunigung*

## Weitere Resourcen

- [OpenCV Haar Cascade Tutorial](https://docs.opencv.org/4.x/db/d28/tutorial_cascade_classifier.html)
- [ROS2 cv_bridge Tutorial](https://github.com/ros-perception/vision_opencv/tree/rolling/cv_bridge)
- [v4l2_camera Dokumentation](https://github.com/ros-drivers/v4l2_camera)

## Lizenz

MIT License - siehe [LICENSE](LICENSE)

## Autoren

- Sebastian Zug (sebastian.zug@informatik.tu-freiberg.de)

---

**Verwendung in der Vorlesung**: Beispiel für Bildverarbeitung mit ROS2 und OpenCV
