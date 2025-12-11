# Übung 1: GNSS-Datenanalyse & ROS 2-Bags

**Zeitaufwand**: 180 Minuten

**Voraussetzungen**: Vorlesungen 1-5

**Abgabe**: [Termin wird bekanntgegeben]


## Szenario: Ihr erster Tag im Praktikum

**Willkommen bei RoboTech Solutions GmbH!**

Sie haben Ihr Praktikum bei einem innovativen Roboterhersteller begonnen, der autonome Inspektionsroboter für Außenbereiche entwickelt. Ihr Team arbeitet gerade an einem neuen GNSS-basierten Navigationssystem.

**Ihre erste Aufgabe**: Gestern hat das Testteam eine Versuchsfahrt auf dem Firmengelände durchgeführt. Der Roboter hat während der Fahrt verschiedene Sensordaten aufgezeichnet – insbesondere von drei GNSS-Quellen:
- Zwei unabhängige GNSS-Antennen (`gnss1`, `gnss2`)
- Ein fusioniertes GNSS-Odometrie-System (`odometry_llh`)

**Ihr Auftrag**: Werten Sie den aufgezeichneten Datensatz aus und erstellen Sie bis zum Ende des Tages ein **technisches Protokoll**, das Ihre Erkenntnisse zusammenfasst. Ihre Teamleiterin möchte wissen:

1. Welche Daten wurden aufgezeichnet? (Sensoren, Frequenzen, Frames)
2. Wie verlief die Trajektorie des Roboters?
3. Wie genau sind die GNSS-Systeme? Gibt es Abweichungen?
4. Wo treten Probleme auf? (Drift, Unsicherheiten, Multipath-Effekte)
5. Bringt die Sensor-Fusion einen Vorteil?

**Hilfsmittel**: Sie haben Zugriff auf die aufgezeichnete ROS2-Bag-Datei und die üblichen ROS2-Tools. Ihre Kolleg:innen haben Ihnen Templates für die Python-Skripte vorbereitet.

**Zeitrahmen**: Sie haben etwa 3 Stunden Zeit. Ihre Teamleiterin erwartet am Ende ein strukturiertes Protokoll mit Plots, Screenshots und Ihrer technischen Einschätzung.

## Lernziele

Nach Abschluss dieser Übung können Sie:

- ROS2-Bag-Dateien inspizieren und abspielen
- Sensordaten mit RViz2 und rqt-Tools visualisieren
- GNSS-Daten auslesen und verarbeiten
- Trajektorien plotten und analysieren
- Genauigkeit und Drift von GNSS-Systemen bewerten
- Python-Skripte für ROS2-Bag-Analyse schreiben
- Technische Dokumentation erstellen

## Datensatz

https://ificloud.xsitepool.tu-freiberg.de/index.php/s/DQqrBKBZw3JBeBm

Laden Sie den Datensatz herunter, entpacken Sie diesen und platzieren Sie ihn im `data/`-Verzeichnis dieser Übung. 

Wenn Sie den Datensatz aus verschiedenen Ordnern referenzieren möchten, können Sie einen symbolischen Link erstellen.



**Relevante Topics für diese Übung**:

| Topic                              | Message Type              | Beschreibung                |
| ---------------------------------- | ------------------------- | --------------------------- |
| `/fixposition/gnss1`               | `sensor_msgs/NavSatFix`   | GNSS-Antenne 1              |
| `/fixposition/gnss2`               | `sensor_msgs/NavSatFix`   | GNSS-Antenne 2              |
| `/fixposition/odometry_llh`        | `sensor_msgs/NavSatFix`   | Fusionierte GNSS-Position   |
| `/fixposition/odometry_enu`        | `nav_msgs/Odometry`       | Fusionierte Odometrie (ENU) |
| `/zed2i_front/.../rgb/.../image`   | `sensor_msgs/Image`       | RGB-Kamerabild              |
| `/zed2i_front/.../point_cloud/...` | `sensor_msgs/PointCloud2` | PointCloud (Stereo)         |
| `/lynx/odometry/wheels`            | `nav_msgs/Odometry`       | Rad-Odometrie               |

### Protokollstruktur

```
exercise_01_gnss_bags/
├── data/
│   └── 20251126_ifi2
└── summmary/
    ├── results/
    │   ├── PROTOKOLL.md                           ⭐ HAUPTABGABE
    │   ├── task_0_0_dataset_exploration.md
    │   ├── task_0_1_tf_analysis.md
    │   ├── frames.pdf
    │   ├── task_0_2_camera_screenshot.png
    │   ├── task_0_2_pointcloud_screenshot.png
    │   ├── task_0_2_observations.md
    │   ├── task_0_3_trajectory_screenshot.png
    │   ├── task_0_3_trajectory_description.md
    │   ├── gnss1_data.csv
    │   ├── gnss2_data.csv
    │   ├── odometry_llh_data.csv
    │   ├── task_1_2_trajectory_comparison.png
    │   ├── task_1_2_distances.txt
    │   ├── task_1_3_sensor_deviation.png
    │   ├── task_1_3_position_covariance.png
    │   ├── task_1_3_statistics.md
    │   └── [optional] task_1_4_*
    ├── solution/
    │   ├── task_1_1_gnss_extractor.py
    │   ├── task_1_2_trajectory_plotter.py
    │   ├── task_1_3_uncertainty_analysis.py
    │   └── [optional] task_1_4_fusion_analysis.py
    └── rviz_configs/
       └── gnss_analysis.rviz
```

> Wer möchte kann auch gern ein Jupyter-Notebook als eine Markdown-Datei zur Analyse verwenden. Achten Sie in diesem Fall darauf, dass alle Ergebnisse (Plots, Screenshots, Auswertungen) auch im Protokoll enthalten sind.

### Protokollinhalt

Sie können Ihre Ergebnisse in beliebiger Reihenfolge erarbeiten, sollten jedoch am Ende ein strukturiertes technisches Protokoll erstellen.

**Datei**: `rsummaryesults/PROTOKOLL.md`

**Struktur**:

```markdown
# Technisches Protokoll: GNSS-Datenanalyse Versuchsfahrt 2024-11-26

**Datum**: [Datum einfügen]
**Bearbeiter**: [Ihr Name]
**Auftraggeber**: RoboTech Solutions GmbH – Abteilung Autonome Navigation

## 1. Executive Summary

[2-3 Sätze: Was wurde untersucht? Was sind die wichtigsten Erkenntnisse? Beschreiben Sie kurz das technische Setup.]

## 2. Datensatz-Übersicht

### 2.1 Aufgezeichnete Sensoren
[Aus Aufgabe 0.0]
- Anzahl Topics: ...
- Topic mit höchster Nachrichtenrate: ...
- Gesamtdauer der Aufzeichnung: ...
- Verwendete Message-Typen: ...

### 2.2 TF-Frame-Hierarchie
[Aus Aufgabe 0.1]
- Anzahl Frames: ...
- Wichtigste Frames und deren Beziehungen: ...
- Screenshot des TF-Baums (frames.pdf)

### 2.3 Sensorische Beobachtungen
[Aus Aufgabe 0.2]
- Beschreibung der Umgebung (PointCloud): ...
- Beschreibung der Umgebung (Kamera): ...
- Sensorreichweite und -qualität: ...

## 3. Trajektorien-Analyse

### 3.1 Verlauf der Testfahrt
[Aus Aufgabe 1.2]
- Zurückgelegte Strecke:
  - GNSS1: ... m
  - GNSS2: ... m
  - Odometry_LLH: ... m
- Routenbeschreibung: ...
- Screenshot der Trajektorien

### 3.2 Abweichungen zwischen Sensoren
[Aus Aufgabe 1.3]
- Mittlere Abweichung GNSS1 vs. GNSS2: ... cm
- Maximale Abweichung: ... cm
- Zeitpunkt/Ort der größten Abweichung: ...
- Mögliche Ursachen: ...

## 4. Genauigkeit und Unsicherheit

### 4.1 Position Covariance
[Aus Aufgabe 1.3]
- Zeitbereiche mit hoher Unsicherheit: ...
- Typische Standardabweichungen: σ_lat = ..., σ_lon = ...
- Korrelation mit Umgebung: ...

### 4.2 Einfluss der Umgebung
- Wo treten die größten Unsicherheiten auf?
- Gibt es sichtbare Ursachen (Gebäude, Bäume, ...)?

## 5. Bewertung der Sensor-Fusion

[Aus Aufgabe 1.4 – falls bearbeitet]
- Ist die fusionierte Lösung stabiler? ...
- Quantitative Metriken (Glattheit, Rauschen): ...
- Wo zeigt sich der Vorteil der Fusion? ...

## 6. Fazit und Empfehlungen

### 6.1 Technische Bewertung
- Sind die GNSS-Systeme zuverlässig genug für autonome Navigation in diesem Umfeld?
- Kritische Bereiche und Einschränkungen: ...

### 6.2 Empfehlungen
- Verbesserungsvorschläge für das Entwicklungsteam: ...
- Sollten zusätzliche Sensoren integriert werden?
- Empfohlene weitere Tests: ...

---

**Erstellt mit**: ROS2 Jazzy, Python 3, Matplotlib
**Analysierter Datensatz**: 20251126_ifi2_0.mcap
```

## Abgabe

**Deadline**: [wird bekanntgegeben]

**Format**: Komprimieren Sie Ihren `exercise_01_gnss_bags/` Ordner:

```bash
cd exercises
tar -czf exercise_01_NACHNAME_VORNAME.tar.gz exercise_01_gnss_bags/
```

## Teil A: Grundlegende ROS2-Werkzeuge

> **Ziel**: Vertrautheit mit ROS2-CLI-Tools und Visualisierung erlangen

### Aufgabe 0.0: Datensatz-Exploration

**Kontext**: Bevor Sie mit der Datenanalyse beginnen, müssen Sie verstehen, welche Daten im Bag-File enthalten sind. Diese explorative Phase ist in jedem Datenanalyse-Projekt der erste Schritt.

**Fragen**:

1. Welches Topic hat die höchste Anzahl an gesendeten Nachrichten? Warum könnte das so sein?
2. Welches Topic hat die höchste Publikationsfrequenz (in Hz)? Messen Sie diese während das Bag abgespielt wird.
3. Welches Topic verursacht das größte Datenvolumen? Berechnen Sie dies aus Nachrichtenanzahl und durchschnittlicher Message-Größe.
4. Welche verschiedenen Message-Typen werden im Datensatz verwendet? Listen Sie mindestens 5 auf und beschreiben Sie kurz deren Verwendungszweck.
5. Vergleichen Sie die Message-Typen von `/fixposition/gnss1` und `/fixposition/odometry_llh`. Was ist der Unterschied? Welche Datenquelle würden Sie als genauer/stabiler einschätzen und warum? Ihr Betreuer verweist Sie dabei auf die [Dokumentation des Sensors](https://docs.fixposition.com/__attachments/776306689/VRTK2_Datasheet_v1.1%201.pdf?inst-v=76318eed-3f39-428c-b1f9-d11cbb01e641)

**Hilfreiche Kommandos**:

```bash
# Bag-Informationen anzeigen
ros2 bag info data/dataset

# Bag abspielen
ros2 bag play data/dataset

# Frequenz messen (in separatem Terminal, während Bag läuft)
ros2 topic hz <topic_name>

# Message-Typen anzeigen
ros2 topic type <topic_name>
ros2 interface show <message_type>

# Beispiel-Nachrichten anschauen
ros2 topic echo <topic_name> --once

# Messen der Message-Größe (in Bytes, während Bag läuft)
ros2 topic bw <topic_name>

# Beispiel: Message-Typ von /fixposition/gnss1 anzeigen
ros2 interface show sensor_msgs/msg/NavSatFix
```

---

### Aufgabe 0.1: TF-Frame-Hierarchie verstehen

**Kontext**: Das Transform-System (TF) in ROS2 beschreibt die räumlichen Beziehungen zwischen verschiedenen Koordinatensystemen (Frames) des Roboters. Kameras, Sensoren und der Roboter selbst haben jeweils eigene Frames. Das Verständnis dieser Hierarchie ist essentiell für die Arbeit mit Sensordaten.

**Fragen**:

1. Auf welchen Frame bezieht sich die PointCloud `/zed2i_front/zed_node_front/point_cloud/cloud_registered`? Lesen Sie den `header.frame_id` aus.
2. Erstellen Sie eine Visualisierung des TF-Baums als PDF. Welche Frames existieren? Listen Sie alle auf.
3. Was ist der Parent-Frame von `zed2i_front_camera_link`? Beschreiben Sie die Transformationskette vom `base_link` zur Kamera.
4. Warum ist es wichtig, dass alle Sensordaten in einem gemeinsamen Referenzframe ausgedrückt werden können?

**Hilfreiche Kommandos**:

```bash
# Bag abspielen
ros2 bag play data/dataset.mcap &

# Frame-ID aus Message auslesen
ros2 topic echo /zed2i_front/zed_node_front/point_cloud/cloud_registered --once | grep frame_id

# TF-Baum als PDF visualisieren
ros2 run tf2_tools view_frames
# Öffnet frames.pdf im aktuellen Verzeichnis

# Transformation zwischen zwei Frames anzeigen
ros2 run tf2_ros tf2_echo <source_frame> <target_frame>

# Statische Transforms anzeigen
ros2 topic echo /tf_static --once
```

### Aufgabe 0.2: Sensordaten visualisieren

**Kontext**: Visualisierung hilft, die Daten zu verstehen und Probleme zu identifizieren. Sie nutzen RViz2.

Starten Sie Rviz2 in einem separaten Terminal, während Sie das Bag abspielen.

**Aufgaben**:

1. **RGB-Kamera**: Zeigen Sie das Live-Videobild `/zed2i_front/zed_node_front/rgb/color/rect/image` an. Was sehen Sie in der Umgebung? Beschreiben Sie die Szene (Gebäude, Vegetation, Straßen, etc.).

2. **PointCloud**: Visualisieren Sie die PointCloud in RViz2. Konfigurieren Sie:
   - Fixed Frame: `zed2i_front_camera_link` oder `base_link`
   - PointCloud2 Display mit RGB8 Color Transformer
   - Geeignete Punktgröße (3-5 Pixel)

3. **Beobachtungen**:
   - Welche Objekte sind in der PointCloud erkennbar?
   - Wie weit reicht die Sensor-Reichweite?
   - Gibt es Bereiche mit schlechter Punktdichte? Wo und warum?

4. **Fixed Frame wechseln**: Was passiert, wenn Sie den Fixed Frame von `base_link` auf `map` ändern? Erklären Sie den Unterschied.

**Hilfreiche Kommandos**:

```bash
# Terminal 1: Bag abspielen
ros2 bag play data/dataset.mcap --rate 0.5 --loop
```

### Aufgabe 0.3: RViz2-Konfiguration speichern und ladn

**Kontext**: Eine gute RViz2-Konfiguration spart Zeit bei wiederholten Analysen. Wir haben ein Konfiguration als `rviz_configs/PointCloud_laser_analysis.rviz` für Sie vorbreitet.

**Aufgabe**: 
1. Starten Sie RViz2 mit der gespeicherten Konfiguration.
2. Ersetzen Sie die Laserscanner Point Cloud durch eine Visualisierung eine Darstellung der ZED-Stereo Kameras.
3. Passen Sie die Konfiguration so an, dass die RGB Bildaten über die Point-Cloud gelegt werden.
4. Speichern Sie Ihre angepasste Konfiguration als `rviz_configs/PointCloud_zed_analysis.rviz`

**Hilfreiche Kommandos**:

```bash
# Starten mit gespeicherter Config
rviz2 -d rviz_configs/PointCloud_laser_analysis.rviz
```

## Teil B: Programmierung und Datenanalyse

> **Ziel**: GNSS-Daten programmatisch auswerten und analysieren

### Aufgabe 1.1: GNSS-Daten extrahieren

**Kontext**: Um quantitative Analysen durchzuführen, müssen Sie die Daten aus dem Bag-File extrahieren und in einem verarbeitbaren Format (CSV) speichern.

**Aufgabe**: Schreiben Sie ein Python-Skript, das folgende Daten extrahiert:

**Zu extrahierende Topics**:
- `/fixposition/gnss1`
- `/fixposition/gnss2`
- `/fixposition/odometry_llh`

**Zu speichernde Felder pro Message**:
- Timestamp (in Sekunden seit Start der Aufzeichnung)
- Latitude (Breitengrad in Dezimalgrad)
- Longitude (Längengrad in Dezimalgrad)
- Altitude (Höhe in Metern)
- Position Covariance (Diagonalelemente: σ_lat, σ_lon, σ_alt)

**Ausgabe**: 3 CSV-Dateien mit Header:
```csv
timestamp,latitude,longitude,altitude,cov_lat,cov_lon,cov_alt
```

**Hilfreiche Code-Snippets**: Siehe `templates/gnss_extractor_template.py`

### Aufgabe 1.2: Trajektorien vergleichen

**Kontext**: Ein visueller Vergleich der drei GNSS-Quellen zeigt, wie gut sie übereinstimmen und wo Abweichungen auftreten.

**Aufgaben**:

1. Lesen Sie die CSV-Dateien aus Aufgabe 1.1
2. Konvertieren Sie Lat/Lon zu einem metrischen Koordinatensystem (verwenden Sie die Flat-Earth-Approximation oder UTM)
3. Erstellen Sie einen 2D-Plot mit allen drei Trajektorien:
   - GNSS1 (blau)
   - GNSS2 (grün)
   - Odometry_LLH (rot)
   - Markieren Sie den Startpunkt
   - Achsenskalierung: gleich (aspect ratio = 'equal')
   - Achsenbeschriftungen: "X [m]", "Y [m]"

4. Berechnen Sie die zurückgelegte Strecke für jede Trajektorie
5. Analysieren Sie: Wo weichen die Trajektorien am stärksten ab? Was könnte die Ursache sein?

**Hilfreiche Formeln**:

```python
# Flat-Earth-Approximation
def latlon_to_meters(lat, lon, lat0, lon0):
    R = 6371000  # Erdradius in Metern
    x = (lon - lon0) * np.cos(np.radians(lat0)) * R * np.pi / 180
    y = (lat - lat0) * R * np.pi / 180
    return x, y

# Streckenlänge
def calculate_distance(x, y):
    dx = np.diff(x)
    dy = np.diff(y)
    distances = np.sqrt(dx**2 + dy**2)
    return np.sum(distances)
```

### Aufgabe 1.3: Genauigkeit und Unsicherheit analysieren

**Kontext**: GNSS-Systeme haben unterschiedliche Genauigkeiten abhängig von Umgebung und Satellitensicht. Die Position Covariance gibt Auskunft über die Messunsicherheit.

**Teil A: Sensor-Abweichungen**

**Aufgaben**:
1. Berechnen Sie die euklidische Distanz zwischen GNSS1 und GNSS2 für jeden Zeitpunkt
2. Erstellen Sie einen Plot: Abweichung [cm] über Zeit [s]
3. Berechnen Sie Statistiken:
   - Mittlere Abweichung
   - Maximale Abweichung
   - Standardabweichung
   - Zeitpunkt der größten Abweichung

4. Interpretation:
   - Wo weichen die Sensoren am stärksten ab?
   - Was könnte die Ursache sein? (Multipath, Abschattung durch Gebäude, etc.)
   - Schauen Sie sich das Video an der entsprechenden Stelle an!

**Teil B: Position Covariance**

**Aufgaben**:
1. Extrahieren Sie die Position Covariance aus `/fixposition/odometry_llh`
2. Berechnen Sie die Standardabweichungen: σ_lat = √(cov_lat), σ_lon = √(cov_lon)
3. Erstellen Sie einen Plot: σ_lat und σ_lon über Zeit
4. Interpretation:
   - Wann ist die GNSS-Genauigkeit am schlechtesten?
   - Korreliert dies mit der Umgebung? (Vergleich mit Kamerabild)
   - Was bedeutet eine hohe Kovarianz für die Navigationssicherheit?

### Aufgabe 1.4: Sensor-Fusion bewerten (Bonus)

**Kontext**: Das fusionierte GNSS-System kombiniert mehrere Sensoren. Bringt dies einen Vorteil gegenüber den einzelnen Antennen?

**Aufgaben**:

1. Vergleichen Sie die "Glattheit" der Trajektorien:
   - Berechnen Sie die Summe der Beschleunigungen (zweite Ableitung der Position)
   - Welche Trajektorie ist am glattesten?

2. Analyse der Rausch-Charakteristik:
   - Berechnen Sie die hochfrequenten Komponenten (z.B. mit FFT)
   - Ist die fusionierte Lösung weniger verrauscht?

3. Visualisierung:
   - Plotten Sie alle drei Trajektorien übereinander
   - Zoomen Sie auf einen Bereich mit hoher Abweichung

4. Bewertung:
   - Wo ist die fusionierte Lösung stabiler?
   - Welche Vorteile bietet die Fusion?
   - Würden Sie der fusionierten Lösung für ein autonomes System vertrauen?

## Hilfe und Ressourcen

**ROS2-Dokumentation**:
- [rosbag2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
- [RViz2 User Guide](https://github.com/ros2/rviz/blob/jazzy/docs/index.md)
- [sensor_msgs/NavSatFix](https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html)
- [TF2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html)

**Python-Libraries**:
- `rosbag2_py`: ROS2-Bag-API
- `matplotlib`: Plotting
- `numpy`: Numerische Berechnungen
- `pandas`: CSV-Verarbeitung (optional)
