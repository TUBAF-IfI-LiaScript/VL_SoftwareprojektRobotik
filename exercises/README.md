# Übungen - Softwareprojekt Robotik

Dieses Verzeichnis enthält alle praktischen Übungsaufgaben zur Vorlesung "Softwareprojekt Robotik".

## Übersicht

_Stand WiSe 2026/27 — Termine nach dem neuen Format (VL, Ü und P à 90 min). Autoren der neuen Übungen: Sebastian Zug & Claude Code (Opus 5.5) mit LiaScript Teaching-Agent._

| Termin   | Material                                                       | Thema                                                   | Status          |
| -------- | -------------------------------------------------------------- | ------------------------------------------------------- | --------------- |
| **Ü 1**  | [uebung_01_linux_shell](uebung_01_linux_shell/)             | OS + Linux Shell, Installation von ROS 2              | neu 2026/27     |
| zu Ü 1   | [installation_ros2](installation_ros2/)                         | ROS 2 Jazzy installieren: gemeinsam in Ü 1, hier zum Nachschlagen und Nachholen | neu 2026/27     |
| **Ü 2**  | [uebung_02_workspace](uebung_02_workspace/)     | Services, Parameter, Actions, `ros2 bag` mit turtlesim  | neu 2026/27     |
| **P 1**  | [praktikum_01_turtlebot](praktikum_01_turtlebot/)               | TurtleBot verbinden, untersuchen, fahren, aufzeichnen   | neu 2026/27     |
| **P 2**  | [praktikum_02_bildkorrektur](praktikum_02_bildkorrektur/)       | Kamera kalibrieren, entzerren, Vogelperspektive; Projektpaket `verkehrswelt` | neu 2026/27 |
| **P 3**  | [praktikum_03_spurerkennung](praktikum_03_spurerkennung/)       | Fahrspur erkennen, Abweichung `/lane/offset`            | neu 2026/27     |

**Bestand 2025/26** (180-min-Blockübungen, werden auf die neuen Ü- und P-Termine verteilt):

| Übung (alt)                                  | Thema                                 | Geplante Verwendung 2026/27                         |
| -------------------------------------------- | ------------------------------------- | --------------------------------------------------- |
| [exercise_01_gnss_bags](exercise_01_gnss_bags/)       | GNSS-Datenanalyse & ROS 2-Bags        | Kandidat für Ü 3 Datenvisualisierung (offen)        |
| [exercise_02_person_detection](exercise_02_person_detection/) | Personendetektion mit YOLOv8          | Block 3 Objekterkennung (offen)                     |
| [exercise_03_line_following](exercise_03_line_following/) ⭐ | Linienverfolgung & Hinderniserkennung | Praktika am TurtleBot (Teil A in P 1; Idee der Linienerkennung in P 3) |

## Datensatz & Hardware

**Übung 1 & 2** basieren auf einem gemeinsamen ROS2-Bag-Datensatz:

**Übung 3** verwendet echte TurtleBot 3 Burger Roboter (Gruppenarbeit, 3–4 Personen pro Roboter).

### Bag-Datensatz (Übung 1 & 2)

https://ificloud.xsitepool.tu-freiberg.de/index.php/s/DQqrBKBZw3JBeBm

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
# ROS-2-Umgebung prüfen
printenv ROS_DISTRO
ros2 doctor

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
