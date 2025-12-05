# Face Detection Workspace - Übersicht

Vollständiger ROS2-Workspace für Echtzeit-Gesichtserkennung in der Vorlesung "Bildverarbeitung".

## 📦 Paket-Struktur

```
FaceDetection/
├── src/face_detector/              # ROS2 Python-Paket
│   ├── face_detector/
│   │   ├── face_detector_node.py   # Hauptnode für Gesichtserkennung
│   │   └── __init__.py
│   ├── launch/
│   │   └── face_detection.launch.py  # Launch-File für komplettes System
│   ├── package.xml                 # Paket-Metadaten
│   └── setup.py                    # Python Setup
│
├── install/                        # Build-Artefakte (generiert)
├── build/                          # Build-Cache (generiert)
├── log/                            # Build-Logs (generiert)
│
├── README.md                       # Vollständige Dokumentation
├── QUICKSTART.md                   # 5-Minuten-Start-Guide
├── OVERVIEW.md                     # Diese Datei
├── build.sh                        # Automatisches Build-Skript
├── test_camera.sh                  # Kamera-Test-Skript
└── .gitignore                      # Git-Ignore-Regeln
```

## 🚀 Komponenten

### 1. face_detector_node.py

**Funktionalität**:
- Subscribed: `/image_raw` (sensor_msgs/Image)
- Published: `/face_detection/image` (sensor_msgs/Image, annotiert)
- Published: `/face_detection/faces` (std_msgs/Int32, Anzahl)

**Algorithmus**:
- OpenCV Haar Cascade Classifier
- Graustufen-Konvertierung für Performance
- Echtzeit-Annotation mit Bounding Boxes

**Parameter**:
- `scale_factor`: Bildpyramiden-Skalierung (1.1-2.0)
- `min_neighbors`: Mindest-Nachbarn für Validierung (3-10)
- `min_size_width/height`: Minimale Gesichtsgröße in Pixel
- `cascade_file`: Optionaler Pfad zu eigenem Cascade

### 2. face_detection.launch.py

**Startet automatisch**:
1. **v4l2_camera_node**: USB-Kamera-Treiber
2. **face_detector_node**: Gesichtserkennung
3. **rqt_image_view**: Bildvisualisierung
4. **rosbag2 record**: Datenaufzeichnung (optional)

**Launch-Parameter**:
- `video_device`: Kamera-Device-Pfad (default: /dev/video0)
- `image_width`: Bildbreite (default: 640)
- `image_height`: Bildhöhe (default: 480)
- `record_bag`: Aufzeichnung ein/aus (default: true)
- `show_image`: Viewer anzeigen (default: true)

## 🔧 Installation

```bash
# 1. Abhängigkeiten
sudo apt install ros-jazzy-v4l2-camera ros-jazzy-rqt-image-view \
                 ros-jazzy-cv-bridge v4l-utils
pip3 install opencv-python

# 2. Build
cd FaceDetection
./build.sh

# 3. Source
source install/setup.bash
```

## 🎯 Verwendung

### Schnellstart

```bash
ros2 launch face_detector face_detection.launch.py
```

### Manuell (Lernzwecke)

```bash
# Terminal 1: Kamera
ros2 run v4l2_camera v4l2_camera_node

# Terminal 2: Detektor
ros2 run face_detector face_detector_node

# Terminal 3: Viewer
ros2 run rqt_image_view rqt_image_view /face_detection/image
```

### Parameter anpassen

```bash
# Höhere Sensitivität
ros2 launch face_detector face_detection.launch.py \
    scale_factor:=1.05 min_neighbors:=3

# Andere Kamera
ros2 launch face_detector face_detection.launch.py \
    video_device:=/dev/video2

# Ohne Recording
ros2 launch face_detector face_detection.launch.py \
    record_bag:=false
```

## 📊 Topics

| Topic | Type | QoS | Beschreibung |
|-------|------|-----|--------------|
| `/image_raw` | sensor_msgs/Image | 10 | Kamera-Input |
| `/face_detection/image` | sensor_msgs/Image | 10 | Annotiertes Bild |
| `/face_detection/faces` | std_msgs/Int32 | 10 | Gesichter-Anzahl |

## 🎓 Didaktischer Nutzen

### Für Studierende

**Lerninhalte**:
1. ROS2-Paket-Struktur (Python)
2. cv_bridge: ROS ↔ OpenCV Integration
3. Launch-Files mit mehreren Nodes
4. Parameter-Tuning für Bildverarbeitung
5. Topic-Visualisierung und Debugging

**Übungsaufgaben**:
- Parameter-Optimierung für verschiedene Szenarien
- Erweiterte Features (z.B. Augen-Detektion)
- Performance-Messung und -Verbesserung
- Eigene Cascade-Classifier trainieren

### Code-Beispiele für Vorlesung

**Beispiel 1: ROS2-Image-Subscriber**
```python
def image_callback(self, msg):
    cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    # ... Verarbeitung ...
    output_msg = self.bridge.cv2_to_imgmsg(result, 'bgr8')
    self.publisher.publish(output_msg)
```

**Beispiel 2: Haar Cascade Face Detection**
```python
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
faces = cascade.detectMultiScale(gray,
    scaleFactor=1.1, minNeighbors=5)
for (x, y, w, h) in faces:
    cv2.rectangle(image, (x,y), (x+w,y+h), (0,255,0), 2)
```

**Beispiel 3: Launch-File mit Parametern**
```python
Node(
    package='face_detector',
    executable='face_detector_node',
    parameters=[{'scale_factor': 1.1}]
)
```

## 🔍 Debugging

### Topics inspizieren

```bash
# Live-Frequenz messen
ros2 topic hz /face_detection/faces

# Daten anzeigen
ros2 topic echo /face_detection/faces

# Node-Graph visualisieren
rqt_graph
```

### Logs prüfen

```bash
# Node-Logs anzeigen
ros2 run face_detector face_detector_node --ros-args --log-level DEBUG

# Build-Logs
cat log/latest_build/face_detector/stdout_stderr.log
```

### Performance messen

```bash
# Bag aufzeichnen und später analysieren
ros2 bag record /face_detection/faces -o performance_test

# FPS berechnen
ros2 topic hz /face_detection/image
```

## 📈 Erweiterungsmöglichkeiten

### Für fortgeschrittene Studierende

1. **Multi-Cascade-Detektion**
   - Gesichter + Augen + Mund gleichzeitig
   - Eigene Cascade-Classifier trainieren

2. **Deep Learning Integration**
   - YOLO, SSD oder andere CNN-basierte Detektoren
   - GPU-Beschleunigung mit TensorRT

3. **Tracking**
   - Gesichter über Frames hinweg verfolgen
   - Kalman-Filter für smooth tracking

4. **3D-Pose-Estimation**
   - Gesichtsausrichtung schätzen
   - Integration mit Stereo-Kameras

5. **Multi-Camera-Setup**
   - Mehrere Kameras parallel
   - Sensor-Fusion

## 📝 Prüfungsfragen

1. Warum wird das Bild zu Graustufen konvertiert?
2. Was bewirkt der `scale_factor` Parameter?
3. Wie funktioniert die cv_bridge?
4. Was ist der Unterschied zwischen `/image_raw` und `/face_detection/image`?
5. Wie würden Sie False Positives reduzieren?

## 🐛 Bekannte Einschränkungen

- **Haar Cascade**: Nur frontale Gesichter, anfällig für Beleuchtung
- **Performance**: CPU-basiert, langsam bei hoher Auflösung
- **False Positives**: Bei komplexen Hintergründen
- **Latenz**: ~50-200ms je nach Hardware

**Lösungsansätze**:
- Deep Learning Modelle (YOLO, SSD)
- GPU-Beschleunigung
- Tiefere Integration mit Stereo-Kameras

## 📚 Ressourcen

- [OpenCV Face Detection Tutorial](https://docs.opencv.org/4.x/db/d28/tutorial_cascade_classifier.html)
- [ROS2 cv_bridge](https://github.com/ros-perception/vision_opencv)
- [v4l2_camera Package](https://github.com/ros-drivers/v4l2_camera)
- [ROS2 Launch Files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)

## 👥 Kontakt

**Vorlesung**: Bildverarbeitung Grundlagen
**Dozent**: Prof. Dr. Sebastian Zug
**Email**: sebastian.zug@informatik.tu-freiberg.de

---

**Version**: 0.1.0
**Letzte Aktualisierung**: 2024-12-05
**Lizenz**: MIT
