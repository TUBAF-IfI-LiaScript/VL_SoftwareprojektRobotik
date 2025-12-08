# Übung 1: GNSS-Datenanalyse & ROS 2-Bags

**Zeitaufwand**: 180 Minuten
**Voraussetzungen**: Vorlesungen 1-5
**Abgabe**: [Termin wird bekanntgegeben]

---

## Lernziele

Nach Abschluss dieser Übung können Sie:

✅ ROS2-Bag-Dateien inspizieren und abspielen
✅ Sensordaten mit RViz2 und rqt-Tools visualisieren
✅ GNSS-Daten auslesen und verarbeiten
✅ Trajektorien plotten und analysieren
✅ Genauigkeit und Drift von GNSS-Systemen bewerten
✅ Python-Nodes für ROS2-Bag-Analyse schreiben

---

## Datensatz

**Pfad**: `data/dataset.mcap` (Symlink zu `/media/sz/Data/20251126_ifi2/20251126_ifi2_0.mcap`)

**Relevante Topics für diese Übung**:

| Topic | Message Type | Frequenz | Beschreibung |
|-------|--------------|----------|--------------|
| `/fixposition/gnss1` | `sensor_msgs/NavSatFix` | ~5 Hz | GNSS-Antenne 1 |
| `/fixposition/gnss2` | `sensor_msgs/NavSatFix` | ~5 Hz | GNSS-Antenne 2 |
| `/fixposition/odometry_llh` | `sensor_msgs/NavSatFix` | ~20 Hz | Fusionierte GNSS-Position |
| `/fixposition/odometry_enu` | `nav_msgs/Odometry` | ~20 Hz | Fusionierte Odometrie (ENU) |
| `/zed2i_front/.../rgb/.../image` | `sensor_msgs/Image` | ~7.5 Hz | RGB-Kamerabild |
| `/zed2i_front/.../point_cloud/...` | `sensor_msgs/PointCloud2` | ~7.5 Hz | PointCloud (Stereo) |
| `/lynx/odometry/wheels` | `nav_msgs/Odometry` | ~100 Hz | Rad-Odometrie |

---

## Teil A: Grundlegende ROS2-Werkzeuge (40 Punkte)

> **Ziel**: Vertrautheit mit ROS2-CLI-Tools und Visualisierung

### Aufgabe 0.1: Bag-Datei inspizieren (5 Punkte)

**Aufgabenstellung**:

1. Nutzen Sie `ros2 bag info` um Informationen über den Datensatz zu erhalten
2. Dokumentieren Sie in `results/task_0_1_bag_info.txt`:
   - Dauer der Aufnahme (in Sekunden)
   - Gesamtzahl der Messages
   - Anzahl der Topics
   - Storage-Format
   - ROS2-Distribution

**Kommandos**:

```bash
cd exercises/exercise_01_gnss_bags
ros2 bag info data/dataset.mcap > results/task_0_1_bag_info.txt
```

**Fragen** (beantworten Sie in `results/task_0_1_answers.md`):

a) Wie viele GNSS-Nachrichten (alle GNSS-Topics zusammen) sind enthalten?
b) Welcher Sensor hat die höchste Nachrichtenrate?
c) Warum ist das MCAP-Format für große Datensätze vorteilhaft?

---

### Aufgabe 0.2: Bag abspielen und visualisieren (20 Punkte)

**Aufgabenstellung**:

Spielen Sie den Bag-Datensatz ab und visualisieren Sie:

1. **RGB-Videosignal** mit `rqt_image_view`
2. **PointCloud** des Frontsensors in RViz2

**Schritt-für-Schritt-Anleitung**:

#### Schritt 1: Terminal-Setup (3 Terminals benötigt)

**Terminal 1**: Bag abspielen
```bash
cd exercises/exercise_01_gnss_bags
ros2 bag play data/dataset.mcap --rate 0.5 --loop
```
*Hinweis*: `--rate 0.5` = halbe Geschwindigkeit, `--loop` = endlos wiederholen

**Terminal 2**: RGB-Bild anzeigen
```bash
ros2 run rqt_image_view rqt_image_view
```
- Im GUI: Wählen Sie `/zed2i_front/zed_node_front/rgb/color/rect/image`
- Beobachten Sie die Umgebung während der Fahrt

**Terminal 3**: RViz2 für PointCloud
```bash
rviz2
```

#### Schritt 2: RViz2 konfigurieren

1. **Fixed Frame setzen**:
   - Links unter "Global Options" → "Fixed Frame"
   - Setzen Sie auf `zed2i_front_camera_link` oder `base_link`

2. **PointCloud hinzufügen**:
   - Klicken Sie "Add" (unten links)
   - Wählen Sie "By topic"
   - Wählen Sie `/zed2i_front/zed_node_front/point_cloud/cloud_registered`
   - **PointCloud2** sollte erscheinen

3. **PointCloud-Darstellung anpassen**:
   - Im PointCloud2-Display:
     - Size (Pixels): `2` - `5`
     - Style: `Points` oder `Squares`
     - Color Transformer: `RGB8` (zeigt echte Farben)
   - Alternative: `AxisColor` oder `Intensity`

4. **Optional: TF-Frames anzeigen**:
   - Add → TF
   - Sie sehen den Transformationsbaum des Roboters

**Dokumentation**:

Erstellen Sie Screenshots und speichern Sie in `results/`:
- `task_0_2_image_view.png` - Screenshot von rqt_image_view
- `task_0_2_rviz_pointcloud.png` - Screenshot von RViz2 mit PointCloud

**Fragen** (in `results/task_0_2_answers.md`):

a) Was sehen Sie in der PointCloud? Beschreiben Sie die Umgebung (Gebäude, Straße, Vegetation)
b) Warum ist die PointCloud farbcodiert? Welche Information wird angezeigt?
c) Was passiert, wenn Sie den Fixed Frame ändern? (Testen Sie `base_link` vs. `map`)

---

### Aufgabe 0.3: Topic-Analyse mit ROS2-CLI-Tools (10 Punkte)

**Aufgabenstellung**:

Analysieren Sie die GNSS-Topics mit ROS2-Kommandozeilen-Tools.

#### a) Topics auflisten

```bash
# Alle Topics anzeigen
ros2 topic list

# Nur GNSS-relevante Topics
ros2 topic list | grep gnss
```

#### b) Topic-Informationen abrufen

```bash
# Details zu einem Topic
ros2 topic info /fixposition/gnss1

# Zeige Message-Typ
ros2 topic type /fixposition/gnss1
```

#### c) Nachrichten anzeigen

```bash
# Erste 5 Nachrichten von gnss1
ros2 topic echo /fixposition/gnss1 --once

# Nur Latitude/Longitude (mit Filter)
ros2 topic echo /fixposition/gnss1 --field latitude,longitude
```

#### d) Frequenz messen

```bash
# Wie oft werden Nachrichten publiziert?
ros2 topic hz /fixposition/gnss1
ros2 topic hz /fixposition/odometry_llh
```

**Dokumentation** (`results/task_0_3_topic_analysis.txt`):

Beantworten Sie:

a) Welche Frequenzen haben die GNSS-Topics (in Hz)?
   - `/fixposition/gnss1`: ______ Hz
   - `/fixposition/gnss2`: ______ Hz
   - `/fixposition/odometry_llh`: ______ Hz

b) Was ist der Message-Typ von `sensor_msgs/NavSatFix`? (Nutzen Sie `ros2 interface show`)

c) Welche QoS-Profile werden für GNSS-Topics verwendet? (Reliability, Durability)

---

### Aufgabe 0.4: RViz2-Konfiguration speichern (5 Punkte)

**Aufgabenstellung**:

Erstellen Sie eine umfassende RViz2-Konfiguration, die Sie für zukünftige Analysen verwenden können.

#### Anzuzeigende Elemente:

1. **PointCloud** (wie in Aufgabe 0.2)
2. **Camera-Bild** (als Image-Display)
3. **TF-Transformationsbaum**
4. **Odometry-Pfad** (visualisiert `/fixposition/odometry_enu`)

#### Schritt-für-Schritt:

1. Öffnen Sie RViz2 (mit laufendem Bag)
2. Fügen Sie hinzu:
   - **PointCloud2**: `/zed2i_front/.../cloud_registered`
   - **Camera**: `/zed2i_front/.../rgb/.../image`
   - **TF**: Zeigt alle Koordinatensysteme
   - **Odometry**: `/fixposition/odometry_enu` → wählen Sie "Odometry" Display

3. Passen Sie die Ansicht an:
   - Setzen Sie Fixed Frame: `map` oder `odom`
   - Zoomen Sie passend
   - Aktivieren Sie Grid (optional)

4. **Speichern**:
   - File → Save Config As...
   - Speichern Sie als `rviz_configs/gnss_analysis.rviz`

**Test**:

```bash
# Starten Sie RViz2 mit Ihrer Config
rviz2 -d rviz_configs/gnss_analysis.rviz
```

---

## Teil B: Programmierung und Datenanalyse (60 Punkte)

> **Ziel**: GNSS-Daten programmatisch auswerten

### Aufgabe 1.1: GNSS-Daten extrahieren (15 Punkte)

**Aufgabenstellung**:

Schreiben Sie ein Python-Skript, das GNSS-Daten aus dem Bag-File extrahiert und als CSV speichert.

**Template**: `templates/gnss_extractor_template.py`

**Anforderungen**:

1. Lesen Sie den Bag-File mit `rosbag2_py`
2. Extrahieren Sie `/fixposition/gnss1`, `/fixposition/gnss2`, `/fixposition/odometry_llh`
3. Speichern Sie folgende Daten pro Message:
   - Timestamp (in Sekunden seit Start)
   - Latitude (Breitengrad)
   - Longitude (Längengrad)
   - Altitude (Höhe)
   - Position Covariance (diagonal: σ_lat, σ_lon, σ_alt)

**Ausgabe**:

3 CSV-Dateien in `results/`:
- `gnss1_data.csv`
- `gnss2_data.csv`
- `odometry_llh_data.csv`

**CSV-Format** (Header):
```
timestamp,latitude,longitude,altitude,cov_lat,cov_lon,cov_alt
```

**Hilfestellung**:

```python
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

# Bag öffnen
storage_options = rosbag2_py.StorageOptions(uri='data/dataset.mcap', storage_id='mcap')
converter_options = rosbag2_py.ConverterOptions('', '')
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

# Topics filtern
topic_types = reader.get_all_topics_and_types()
type_map = {t.name: t.type for t in topic_types}

# Nachrichten lesen
while reader.has_next():
    topic, data, timestamp = reader.read_next()
    if topic == '/fixposition/gnss1':
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        # Verarbeiten Sie msg.latitude, msg.longitude, etc.
```

**Abgabe**: `solution/task_1_1_gnss_extractor.py`

---

### Aufgabe 1.2: GNSS-Trajektorie plotten (15 Punkte)

**Aufgabenstellung**:

Visualisieren Sie die Trajektorien aller drei GNSS-Quellen in einem Diagramm.

**Template**: `templates/gnss_plotter_template.py`

**Anforderungen**:

1. Lesen Sie die CSV-Dateien aus Aufgabe 1.1
2. Konvertieren Sie Lat/Lon zu einem metrischen Koordinatensystem (z.B. UTM oder relative Meter)
3. Plotten Sie alle drei Trajektorien in einem 2D-Plot (x/y)
4. Berechnen Sie die Gesamtstrecke für jede Trajektorie

**Hinweise**:

- **Konvertierung**: Nutzen Sie `pyproj` oder eine einfache Näherung:
  ```python
  import numpy as np

  def latlon_to_meters(lat, lon, lat0, lon0):
      """Konvertiert Lat/Lon zu relativen Metern (Flat-Earth-Approximation)"""
      R = 6371000  # Erdradius in Metern
      x = (lon - lon0) * np.cos(np.radians(lat0)) * R * np.pi / 180
      y = (lat - lat0) * R * np.pi / 180
      return x, y
  ```

- **Streckenlänge**:
  ```python
  def calculate_distance(x, y):
      """Berechnet Gesamtstrecke"""
      dx = np.diff(x)
      dy = np.diff(y)
      distances = np.sqrt(dx**2 + dy**2)
      return np.sum(distances)
  ```

**Plot-Anforderungen**:

- Titel: "GNSS-Trajektorie Vergleich"
- Achsenbeschriftungen: "X [m]", "Y [m]"
- Legende mit allen drei Quellen
- Gleiche Achsenskalierung (aspect ratio = 'equal')
- Startpunkt markiert

**Ausgabe**:

- Plot: `results/task_1_2_gnss_trajectory.png`
- Streckenlängen in `results/task_1_2_distances.txt`:
  ```
  GNSS1: 234.5 m
  GNSS2: 235.1 m
  Odometry_LLH: 236.8 m
  ```

**Abgabe**: `solution/task_1_2_gnss_plotter.py`

---

### Aufgabe 1.3: Drift und Genauigkeitsanalyse (20 Punkte)

**Aufgabenstellung**:

Analysieren Sie die Abweichungen zwischen den GNSS-Sensoren und die zeitliche Entwicklung der Positionsgenauigkeit.

#### Teil a) Sensor-Abweichungen (10 Punkte)

1. Berechnen Sie die euklidische Distanz zwischen `gnss1` und `gnss2` für jeden Zeitpunkt
2. Plotten Sie die Abweichung über Zeit
3. Berechnen Sie Statistiken:
   - Mittlere Abweichung
   - Maximale Abweichung
   - Standardabweichung

**Fragen**:
- Wo weichen die Sensoren am stärksten ab? (Zeitpunkt)
- Was könnte die Ursache sein? (Multipath, Abschattung?)

#### Teil b) Position Covariance (10 Punkte)

1. Extrahieren Sie die Position Covariance aus `/fixposition/odometry_llh`
2. Plotten Sie σ_lat und σ_lon über Zeit (Wurzel der diagonalen Elemente)
3. Identifizieren Sie Zeitbereiche mit hoher Unsicherheit

**Fragen**:
- Wann ist die GNSS-Genauigkeit am schlechtesten?
- Korreliert dies mit der Umgebung? (Schauen Sie sich das Video an!)

**Ausgabe**:

- `results/task_1_3_sensor_deviation.png` - Plot der Abweichung gnss1 vs. gnss2
- `results/task_1_3_covariance.png` - Plot der Unsicherheit über Zeit
- `results/task_1_3_statistics.txt` - Statistische Auswertung

**Abgabe**: `solution/task_1_3_drift_analysis.py`

---

### Aufgabe 1.4: Vergleich Fusion vs. Raw-GNSS (10 Punkte, Bonus)

**Aufgabenstellung**:

Vergleichen Sie die fusionierte GNSS-Lösung (`/fixposition/odometry_llh`) mit den rohen GNSS-Daten (`gnss1`, `gnss2`).

#### Analyse:

1. Berechnen Sie die "Glattheit" der Trajektorie (z.B. Summe der Beschleunigungen)
2. Vergleichen Sie die Rausch-Charakteristik (Hochfrequenz-Komponenten)
3. Visualisieren Sie beide Trajektorien übereinander

**Fragen**:
- Wo ist die fusionierte Lösung stabiler?
- Welche Vorteile bietet die Fusion?
- Würden Sie der fusionierten Lösung vertrauen?

**Bonus**: Visualisieren Sie in RViz2 (als Marker-Array)

**Ausgabe**:

- `results/task_1_4_fusion_comparison.png`
- `results/task_1_4_analysis.md` - Schriftliche Analyse (min. 200 Wörter)

**Abgabe**: `solution/task_1_4_fusion_analysis.py`

---

## Abgabe

**Deadline**: [wird bekanntgegeben]

**Format**:

Komprimieren Sie Ihren `exercise_01_gnss_bags/` Ordner:

```bash
cd exercises
tar -czf exercise_01_NACHNAME_VORNAME.tar.gz exercise_01_gnss_bags/
```

**Enthaltene Dateien**:

```
exercise_01_gnss_bags/
├── results/
│   ├── task_0_1_bag_info.txt
│   ├── task_0_1_answers.md
│   ├── task_0_2_image_view.png
│   ├── task_0_2_rviz_pointcloud.png
│   ├── task_0_2_answers.md
│   ├── task_0_3_topic_analysis.txt
│   ├── gnss1_data.csv
│   ├── gnss2_data.csv
│   ├── odometry_llh_data.csv
│   ├── task_1_2_gnss_trajectory.png
│   ├── task_1_2_distances.txt
│   ├── task_1_3_sensor_deviation.png
│   ├── task_1_3_covariance.png
│   ├── task_1_3_statistics.txt
│   └── [optional] task_1_4_*
├── solution/
│   ├── task_1_1_gnss_extractor.py
│   ├── task_1_2_gnss_plotter.py
│   ├── task_1_3_drift_analysis.py
│   └── [optional] task_1_4_fusion_analysis.py
└── rviz_configs/
    └── gnss_analysis.rviz
```

**Upload**: OPAL-Kurs

---

## Bewertung

| Aufgabe | Punkte | Kategorie |
|---------|--------|-----------|
| 0.1 Bag-Inspektion | 5 | ROS2-Tools |
| 0.2 Visualisierung | 20 | ROS2-Tools |
| 0.3 Topic-Analyse | 10 | ROS2-Tools |
| 0.4 RViz-Config | 5 | ROS2-Tools |
| 1.1 Datenextraktion | 15 | Programmierung |
| 1.2 Trajektorie-Plot | 15 | Programmierung |
| 1.3 Drift-Analyse | 20 | Analyse |
| 1.4 Fusion-Vergleich | +10 | Bonus |
| **Gesamt** | **100** (+10) | |

**Bestehen**: 50 Punkte

---

## Hilfe und Ressourcen

**ROS2-Dokumentation**:
- [rosbag2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
- [RViz2 User Guide](https://github.com/ros2/rviz/blob/jazzy/docs/index.md)
- [sensor_msgs/NavSatFix](https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html)

**Python-Libraries**:
- `rosbag2_py`: ROS2-Bag-API
- `matplotlib`: Plotting
- `numpy`: Numerische Berechnungen
- `pandas`: CSV-Verarbeitung (optional)

**Kommandos-Cheatsheet**: Siehe `../README.md`

---

**Viel Erfolg!** 🚀
