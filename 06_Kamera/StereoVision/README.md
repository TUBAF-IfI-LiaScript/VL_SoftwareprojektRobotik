# Stereo Vision - ROS 2 Beispiel

Dieses Paket demonstriert die Verwendung von Stereo-Vision zur Erstellung von 3D-Punktwolken in ROS 2.

## Übersicht

Das Paket enthält:

- **Kamera-Kalibrierung**: Bestimmung intrinsischer und extrinsischer Parameter
- **Stereo-Verarbeitung**: Disparitätsberechnung und 3D-Rekonstruktion
- **PointCloud2-Publishing**: Veröffentlichung von 3D-Daten in ROS 2
- **Praktische Beispiele**: Hinderniserkennung, Bodenebenen-Extraktion

## Voraussetzungen

```bash
# OpenCV mit Python-Bindings
sudo apt install python3-opencv

# ROS 2 Image-Pakete
sudo apt install ros-${ROS_DISTRO}-cv-bridge
sudo apt install ros-${ROS_DISTRO}-image-transport

# Sensor-Messages
sudo apt install ros-${ROS_DISTRO}-sensor-msgs-py

# Visualisierung
sudo apt install ros-${ROS_DISTRO}-rviz2

# Optional: Open3D für erweiterte Punktwolken-Verarbeitung
pip3 install open3d
```

## Paketstruktur

```
StereoVision/
├── src/stereo_vision/
│   ├── camera_calibration.py      # Kamera-Kalibrierung
│   ├── stereo_to_pointcloud.py    # Stereo → PointCloud2
│   ├── pointcloud_processor.py    # PointCloud2 verarbeiten
│   ├── obstacle_detector.py       # Hinderniserkennung
│   └── __init__.py
├── launch/
│   └── stereo_vision.launch.py    # Launch-File
├── config/
│   └── stereo_params.yaml         # Parameter
├── calibration/
│   └── README.md                  # Kalibrieranleitung
├── package.xml
├── setup.py
└── README.md
```

## Installation

1. **Workspace erstellen**:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. **Paket kopieren**:

```bash
cp -r StereoVision ~/ros2_ws/src/
```

3. **Dependencies installieren**:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. **Build**:

```bash
cd ~/ros2_ws
colcon build --packages-select stereo_vision
source install/setup.bash
```

## Verwendung

### 1. Kamera-Kalibrierung

**Vorbereitung:**
- Drucken Sie das Schachbrettmuster aus `calibration/checkerboard.pdf` auf A4
- Kleben Sie es auf eine steife Oberfläche

**Kalibrierung durchführen:**

```bash
# Einzelne Kamera
ros2 run stereo_vision camera_calibration \
    --video-device /dev/video0 \
    --output camera_left.npz

# Stereo-Kalibrierung (nachdem beide Kameras einzeln kalibriert wurden)
ros2 run stereo_vision stereo_calibration \
    --left-device /dev/video0 \
    --right-device /dev/video2 \
    --left-calib camera_left.npz \
    --right-calib camera_right.npz \
    --output stereo_calib.npz
```

### 2. Stereo-Vision starten

```bash
ros2 launch stereo_vision stereo_vision.launch.py
```

**Topics:**
- `/stereo/left/image_raw` - Linkes Kamerabild
- `/stereo/right/image_raw` - Rechtes Kamerabild
- `/stereo/disparity` - Disparitätskarte
- `/stereo/pointcloud` - 3D-Punktwolke (PointCloud2)

### 3. Visualisierung in RViz2

```bash
rviz2
```

**RViz2-Konfiguration:**
1. Add → PointCloud2
2. Topic: `/stereo/pointcloud`
3. Fixed Frame: `camera_link`
4. Style: `Points` oder `Flat Squares`
5. Size: `0.01`
6. Color Transformer: `RGB8`

### 4. PointCloud2 verarbeiten

```bash
# Grundlegende Verarbeitung
ros2 run stereo_vision pointcloud_processor

# Hinderniserkennung
ros2 run stereo_vision obstacle_detector

# Parameter anpassen
ros2 run stereo_vision obstacle_detector \
    --ros-args \
    -p min_distance:=0.5 \
    -p max_distance:=3.0 \
    -p detection_zone_width:=1.0
```

## Kalibrierungsdaten

Nach der Kalibrierung werden folgende Dateien erstellt:

**Einzelkamera-Kalibrierung** (`camera_left.npz`, `camera_right.npz`):
- `K`: Kamera-Matrix (3×3)
- `dist`: Verzerrungskoeffizienten (5×1)

**Stereo-Kalibrierung** (`stereo_calib.npz`):
- `K1`, `K2`: Kamera-Matrizen
- `dist1`, `dist2`: Verzerrungskoeffizienten
- `R`: Rotation zwischen Kameras (3×3)
- `T`: Translation zwischen Kameras (3×1)
- `Q`: Reprojektionsmatrix (4×4)
- `map1x`, `map1y`, `map2x`, `map2y`: Remap-Maps für Rektifizierung

## Parameter

Wichtige Parameter in `config/stereo_params.yaml`:

```yaml
stereo_matcher:
  min_disparity: 0
  num_disparities: 160      # Muss durch 16 teilbar sein
  block_size: 5             # Ungerade Zahl, typisch 3-11
  uniqueness_ratio: 10      # 5-15
  speckle_window_size: 100
  speckle_range: 32

depth_filter:
  min_depth: 0.1            # Meter
  max_depth: 10.0           # Meter

obstacle_detection:
  zone_x_min: -0.5          # Meter (links)
  zone_x_max: 0.5           # Meter (rechts)
  zone_y_min: -0.3          # Meter (unten)
  zone_y_max: 0.3           # Meter (oben)
  zone_z_max: 2.0           # Meter (voraus)
  min_points: 100           # Minimale Punktzahl für Hindernis
```

## Troubleshooting

### Problem: Keine Bilder von Kameras

```bash
# Liste verfügbare Kameras
v4l2-ctl --list-devices

# Teste Kamera
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
ros2 run rqt_image_view rqt_image_view
```

### Problem: Schlechte Disparitätskarte

**Lösungen:**
1. Verbessere Beleuchtung (gleichmäßiges, helles Licht)
2. Erhöhe `block_size` (mehr Glättung)
3. Passe `num_disparities` an (abhängig von Szenen-Tiefe)
4. Kalibrierung wiederholen

### Problem: Zu wenige Punkte in PointCloud

**Lösungen:**
1. Reduziere `min_depth` und erhöhe `max_depth`
2. Verbessere Textur in der Szene
3. Passe Kamera-Ausrichtung an (parallel)

### Problem: Ungenaue Tiefen

**Ursachen:**
- Schlechte Kalibrierung → neu kalibrieren
- Baseline zu klein → Kameras weiter auseinander
- Falsche `Q`-Matrix → Stereo-Kalibrierung prüfen

## Beispiele

### Beispiel 1: Einfache Tiefenmessung

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np

class DepthMeasurement(Node):
    def __init__(self):
        super().__init__('depth_measurement')
        self.sub = self.create_subscription(
            PointCloud2, '/stereo/pointcloud', self.callback, 10
        )

    def callback(self, msg):
        points = np.array(list(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        )))

        if len(points) > 0:
            # Mittlere Tiefe (Z-Koordinate)
            mean_depth = np.mean(points[:, 2])
            self.get_logger().info(f'Mittlere Tiefe: {mean_depth:.2f}m')

def main():
    rclpy.init()
    node = DepthMeasurement()
    rclpy.spin(node)
    rclpy.shutdown()
```

### Beispiel 2: Objektzählung

```python
from sklearn.cluster import DBSCAN

class ObjectCounter(Node):
    def __init__(self):
        super().__init__('object_counter')
        self.sub = self.create_subscription(
            PointCloud2, '/stereo/pointcloud', self.count_objects, 10
        )

    def count_objects(self, msg):
        points = np.array(list(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        )))

        if len(points) < 100:
            return

        # Clustering mit DBSCAN
        clustering = DBSCAN(eps=0.1, min_samples=50).fit(points)

        num_objects = len(set(clustering.labels_)) - (1 if -1 in clustering.labels_ else 0)

        self.get_logger().info(f'Erkannte Objekte: {num_objects}')
```

## Weiterführende Ressourcen

- OpenCV Stereo Vision: https://docs.opencv.org/master/dd/d53/tutorial_py_depthmap.html
- ROS 2 PointCloud2: https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud2.html
- Stereo-Kalibrierung: https://docs.opencv.org/master/d9/d0c/group__calib3d.html

## Lizenz

Dieses Paket ist Teil der Vorlesung "Softwareprojekt Robotik" an der TU Bergakademie Freiberg.
