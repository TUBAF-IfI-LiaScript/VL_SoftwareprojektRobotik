# Übungen - Softwareprojekt Robotik

Dieses Verzeichnis enthält alle praktischen Übungsaufgaben zur Vorlesung "Softwareprojekt Robotik".

## Übersicht

| Übung                                        | Thema                                 | Nach VL | Dauer   | Schwerpunkt                                  |
| -------------------------------------------- | ------------------------------------- | ------- | ------- | -------------------------------------------- |
| **[Übung 1](exercise_01_gnss_bags/)**        | GNSS-Datenanalyse & ROS 2-Bags        | VL 5    | 180 min | ROS2-Tools, Bagfiles, GNSS-Trajektorie       |
| **[Übung 2](exercise_02_person_detection/)** | Personendetektion mit YOLOv8          | VL 8    | 180 min | Objekterkennung, QoS Parameter, Launch-Files |
| **[Übung 3](exercise_03_path_tracking/)**    | Pfadverfolgung & Regelgüte-Evaluation | VL 12   | 180 min | Navigation, Sensorfusion, Systemanalyse      |

## Datensatz

Alle Übungen basieren auf einem gemeinsamen ROS2-Bag-Datensatz:

**Pfad**: `/media/sz/Data/20251126_ifi2/20251126_ifi2_0.mcap`

**Details**:
- Plattform: Clearpath Lynx (mobile Roboterplattform)
- Dauer: ~117 Sekunden
- Größe: 8.8 GB
- ROS2 Distro: Jazzy
- Format: MCAP

**Verfügbare Sensoren**:
- 🛰️ GNSS (Fixposition Fusion System): 3 Topics
- 📷 Stereo-Kamera (ZED 2i): RGB, Depth, PointCloud, Objekterkennung
- 📡 Lidar: SICK (vorne/hinten), Livox
- 🧭 IMU: Inertial Measurement Unit
- 🔄 Odometrie: Rad-Encoder + fusionierte Odometrie
- 🕹️ Joystick-Steuerung

## Vorbereitung

### ROS2-Installation prüfen

```bash
# ROS2-Version prüfen
ros2 --version

# Wichtige Pakete installieren (falls nicht vorhanden)
sudo apt install ros-jazzy-rqt-image-view \
                 ros-jazzy-rviz2 \
                 ros-jazzy-rosbag2-storage-mcap \
                 python3-matplotlib \
                 python3-numpy
```

### Datenzugriff einrichten

Jede Übung enthält einen Symlink zum Hauptdatensatz. Falls der Datensatz an einem anderen Ort liegt:

```bash
# Symlink aktualisieren (Beispiel für Übung 1)
cd exercises/exercise_01_gnss_bags/data
ln -sf /pfad/zum/datensatz/20251126_ifi2_0.mcap dataset.mcap
```

## Arbeitsweise

1. **Lesen Sie die README.md** in jedem Übungsordner
2. **Starten Sie mit den Templates** im `templates/` Verzeichnis
3. **Orientieren Sie sich an den Beispielen**, aber vermeiden Sie das direkte Kopieren
4. **Testen Sie Ihren Code** mit dem bereitgestellten Datensatz
5. **Dokumentieren Sie Ihre Ergebnisse** (Plots, Analysen, Erkenntnisse)

## Bewertung

- **Übung 1**: Vorbereitung für Bildverarbeitung und GNSS-Verständnis
- **Übung 2**: Kernkompetenz für autonome Navigation
- **Übung 3**: Integration aller Konzepte

**Abgabe**: Details werden in der jeweiligen Übung spezifiziert.

## Tipps

💡 **ROS2-Bag-Grundlagen**:
```bash
# Bag-Info anzeigen
ros2 bag info dataset.mcap

# Bag abspielen (langsamer)
ros2 bag play dataset.mcap --rate 0.5

# Nur bestimmte Topics abspielen
ros2 bag play dataset.mcap --topics /fixposition/gnss1 /fixposition/gnss2

# Loop-Modus
ros2 bag play dataset.mcap --loop
```

💡 **Visualisierung**:
```bash
# Bilder anzeigen
ros2 run rqt_image_view rqt_image_view

# RViz2 starten
rviz2

# Topic-Graph anzeigen
rqt_graph
```

💡 **Python-Entwicklung**:
```bash
# Workspace erstellen
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Paket erstellen
ros2 pkg create --build-type ament_python my_analysis

# Bauen und sourcen
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Hilfe und Ressourcen

- **ROS2 Dokumentation**: https://docs.ros.org/en/jazzy/
- **ROS2 Tutorials**: https://docs.ros.org/en/jazzy/Tutorials.html
- **Vorlesungsmaterialien**: `../`
- **OPAL-Kurs**: [Link zum OPAL-Kurs]

---

**Viel Erfolg!** 🚀
