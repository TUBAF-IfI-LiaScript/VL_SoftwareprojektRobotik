# Quick Start - Übung 1

Schnellanleitung zum Starten der Übung.

## 1. Vorbereitung (5 Minuten)

### a) Verzeichnis prüfen

```bash
cd exercises/exercise_01_gnss_bags
ls -la data/dataset.mcap  # Symlink sollte existieren
```

Falls Symlink fehlt:
```bash
ln -sf /media/sz/Data/20251126_ifi2/20251126_ifi2_0.mcap data/dataset.mcap
```

### b) Dependencies installieren

```bash
# ROS2 Jazzy + Tools
sudo apt install ros-jazzy-rqt-image-view \
                 ros-jazzy-rviz2 \
                 ros-jazzy-rosbag2-storage-mcap

# Python-Pakete
pip3 install matplotlib numpy
```

## 2. Aufgabe 0.1: Bag inspizieren (5 Minuten)

```bash
# Bag-Info anzeigen und speichern
ros2 bag info data/dataset.mcap > results/task_0_1_bag_info.txt

# Anschauen
cat results/task_0_1_bag_info.txt
```

**Fragen beantworten in**: `results/task_0_1_answers.md`

## 3. Aufgabe 0.2: Visualisierung (30 Minuten)

### Terminal 1: Bag abspielen

```bash
ros2 bag play data/dataset.mcap --rate 0.5 --loop
```

### Terminal 2: Bild anzeigen

```bash
ros2 run rqt_image_view rqt_image_view
```
- Wähle Topic: `/zed2i_front/zed_node_front/rgb/color/rect/image`
- Screenshot machen → `results/task_0_2_image_view.png`

### Terminal 3: RViz2 starten

```bash
rviz2 -d rviz_configs/gnss_analysis.rviz
```

**Oder manuell konfigurieren:**

```bash
rviz2
```

1. Fixed Frame: `zed2i_front_camera_link`
2. Add → PointCloud2 → Topic: `/zed2i_front/zed_node_front/point_cloud/cloud_registered`
3. PointCloud2-Einstellungen:
   - Size: 3
   - Color Transformer: RGB8
4. Screenshot → `results/task_0_2_rviz_pointcloud.png`

## 4. Aufgabe 0.3: Topics analysieren (15 Minuten)

```bash
# Topics auflisten
ros2 topic list | grep gnss

# Topic-Details
ros2 topic info /fixposition/gnss1

# Nachrichten anzeigen
ros2 topic echo /fixposition/gnss1 --once

# Frequenz messen (Bag muss laufen!)
ros2 topic hz /fixposition/gnss1
ros2 topic hz /fixposition/odometry_llh

# Message-Typ untersuchen
ros2 interface show sensor_msgs/msg/NavSatFix
```

**Ergebnisse dokumentieren in**: `results/task_0_3_topic_analysis.txt`

## 5. Aufgabe 1.1: GNSS-Daten extrahieren (30 Minuten)

```bash
# Template kopieren
cp templates/gnss_extractor_template.py solution/task_1_1_gnss_extractor.py

# Bearbeiten
code solution/task_1_1_gnss_extractor.py  # oder nano/vim

# Ausführen
python3 solution/task_1_1_gnss_extractor.py

# Prüfen
ls -lh results/*.csv
head -5 results/gnss1_data.csv
```

**Erwartete Ausgabe:**
```
results/gnss1_data.csv
results/gnss2_data.csv
results/odometry_llh_data.csv
```

## 6. Aufgabe 1.2: Trajektorien plotten (30 Minuten)

```bash
# Template kopieren
cp templates/gnss_plotter_template.py solution/task_1_2_gnss_plotter.py

# Bearbeiten und ausführen
python3 solution/task_1_2_gnss_plotter.py

# Ergebnis anschauen
eog results/task_1_2_gnss_trajectory.png
cat results/task_1_2_distances.txt
```

## 7. Weitere Aufgaben

Siehe [README.md](README.md) für:
- Aufgabe 1.3: Drift-Analyse
- Aufgabe 1.4: Fusion-Vergleich (Bonus)

## Häufige Probleme

### Problem: "No module named 'rosbag2_py'"

```bash
# ROS2 Environment sourcen
source /opt/ros/jazzy/setup.bash
```

### Problem: "Storage plugin 'mcap' could not be found"

```bash
sudo apt install ros-jazzy-rosbag2-storage-mcap
```

### Problem: Bag wird nicht abgespielt

```bash
# Prüfen, ob Symlink funktioniert
ls -la data/dataset.mcap

# Direkter Pfad (falls Symlink nicht funktioniert)
ros2 bag play /media/sz/Data/20251126_ifi2/20251126_ifi2_0.mcap
```

### Problem: RViz zeigt PointCloud nicht

- Fixed Frame prüfen (sollte `zed2i_front_camera_link` sein)
- Topic-Name korrekt? (Tab-Vervollständigung nutzen!)
- Bag läuft? (`ros2 topic list` sollte Topics anzeigen)

## Nützliche Kommandos

```bash
# Alle verfügbaren Topics anzeigen
ros2 topic list

# Topic-Typ herausfinden
ros2 topic type /fixposition/gnss1

# Message-Definition anzeigen
ros2 interface show sensor_msgs/msg/NavSatFix

# Graph der Node-Verbindungen
rqt_graph

# Nur bestimmte Topics abspielen
ros2 bag play data/dataset.mcap --topics /fixposition/gnss1 /fixposition/gnss2
```

## Zeitplanung (Empfehlung)

| Aufgabe | Zeit | Typ |
|---------|------|-----|
| 0.1 Bag-Info | 10 min | CLI |
| 0.2 Visualisierung | 30 min | RViz/Tools |
| 0.3 Topic-Analyse | 20 min | CLI |
| 0.4 RViz-Config | 10 min | Config |
| **Pause** | 10 min | ☕ |
| 1.1 Extraktion | 40 min | Python |
| 1.2 Plotten | 40 min | Python |
| 1.3 Drift-Analyse | 30 min | Python |
| 1.4 Bonus | +20 min | Python |

**Gesamt**: ~180 Minuten (3 Stunden)

---

**Viel Erfolg!** 🚀
