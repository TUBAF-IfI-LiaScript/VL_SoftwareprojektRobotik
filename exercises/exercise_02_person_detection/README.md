# Übung 2: Personendetektion mit YOLOv8 – Vergleich mit ZED-Objekterkennung

**Zeitaufwand**: 120 Minuten

**Voraussetzungen**: Vorlesungen 1-8, Übung 1

## Szenario

Die ZED2i-Stereokamera verfügt über eine integrierte Objekterkennung. Ihre Aufgabe ist es, diese mit YOLOv8 zu vergleichen und einen Vergleichsplot zu erstellen.

**Ziel**: Vergleichen Sie YOLO und ZED anhand der erkannten Personenanzahl pro Frame.

## Lernziele

- ROS 2 Launch-Files erstellen
- YOLOv8 in ROS 2-Nodes einbinden
- QoS-Profile verstehen (Quality of Service)
- Daten aus CSV laden und visualisieren

## Datensatz

**Gleicher Datensatz wie in Übung 1:**

https://ificloud.xsitepool.tu-freiberg.de/index.php/s/DQqrBKBZw3JBeBm

```bash
cd exercises/exercise_02_person_detection
ln -s ../exercise_01_gnss_bags/data data
```

**Relevante Topics**:

| Topic | Message Type | Count |
|-------|--------------|-------|
| `/zed2i_front/zed_node_front/rgb/color/rect/image` | `sensor_msgs/Image` | 870 |
| `/zed2i_front/zed_node_front/obj_det/objects` | `zed_msgs/ObjectsStamped` | 1305 |

## Vorbereitung: Abhängigkeiten installieren

```bash
# 1. ZED Message-Definitionen
sudo apt install ros-jazzy-zed-msgs

# 2. PyTorch CPU-only
pip install --user torch torchvision --index-url https://download.pytorch.org/whl/cpu

# 3. YOLOv8
pip install --user ultralytics

# 4. OpenCV-Bridge
sudo apt install ros-jazzy-cv-bridge

# 5. Matplotlib
pip install --user matplotlib
```

**Installation prüfen**:

```bash
ros2 interface show zed_msgs/msg/ObjectsStamped
python3 -c "from ultralytics import YOLO; print('OK')"
```

---

## Architektur

```
┌─────────────────┐     ┌──────────────────────────┐
│   rosbag play   │     │    person_detector       │
│                 │     │    (ROS 2 Node)          │
│ /rgb/image ─────┼────►│  YOLOv8 → CSV            │──► results/yolo_detections.csv
│                 │     └──────────────────────────┘
│                 │
│                 │     ┌──────────────────────────┐
│ /obj_det/objects├────►│  zed_extractor_node      │──► results/zed_detections.csv
│                 │     │  (ROS 2 Node)            │
└─────────────────┘     └──────────────────────────┘

                        ┌──────────────────────────┐
yolo_detections.csv ───►│    analyze_results.py    │──► comparison_plot.png
zed_detections.csv  ───►│                          │
                        └──────────────────────────┘
```

---

## Aufgabe 1: Launch-File für ZED-Extraktor (20 min)

**Datei**: `person_detector/launch/zed_extractor.launch.py`

Der ZED-Extraktor-Node (`zed_extractor_node.py`) ist bereits implementiert. Ihre Aufgabe ist es, ein Launch-File zu erstellen.

### Lernziel

- Launch-Files mit `DeclareLaunchArgument` und `LaunchConfiguration` verstehen
- Node-Parameter über Launch-Files konfigurieren

### TODO

Vervollständigen Sie die drei TODO-Bereiche im Launch-File:

1. **TODO 1**: `DeclareLaunchArgument` für `output_csv` mit Default `'results/zed_detections.csv'`
2. **TODO 2**: `Node` mit package, executable, name und parameters
3. **TODO 3**: `LaunchDescription` mit beiden Elementen

### Testen

```bash
# Paket bauen
colcon build --packages-select person_detector
source install/setup.bash

# Terminal 1: Bag abspielen
ros2 bag play data/20251126_ifi2

# Terminal 2: ZED-Extraktor starten
ros2 launch person_detector zed_extractor.launch.py

# Ctrl+C zum Beenden - CSV wird gespeichert
```

**Erwartete Ausgabe**:
```
[INFO] ZED Extractor gestartet.
[INFO] Warte auf ZED-Objektdaten...
[INFO] Frame 50: 2 Personen erkannt
...
[INFO] Gespeichert: 1305 Frames
```

---

## Aufgabe 2: YOLO-Personendetektion (45 min)

**Datei**: `person_detector/person_detector/detector_node.py`

### Aufgabe 2.1: count_persons_yolo() implementieren

Vervollständigen Sie die Funktion `count_persons_yolo()` (ab Zeile 182).

**Schritte**:
1. ROS-Image zu OpenCV konvertieren (Tipp: `self.bridge.imgmsg_to_cv2()`)
2. YOLO-Inferenz durchführen (Tipp: `self.model()`)
3. Personen zählen: Iteriere über `results[0].boxes` und filtere nach Klasse und Konfidenz
4. Boxes sammeln: Speichere `xyxy`-Koordinaten und Konfidenz für jede erkannte Person

**Hinweise**:
- COCO class_id 0 = Person
- `self.confidence_threshold` ist bereits als Parameter definiert (default: 0.5)
- Die Funktion gibt ein Tuple zurück: `(anzahl, liste_der_boxes)`
- Jede Box hat Attribute: `box.cls[0]` (Klasse), `box.conf[0]` (Konfidenz), `box.xyxy[0]` (Koordinaten)

### Aufgabe 2.2: QoS-Profile verstehen

Im Code finden Sie einen Kommentarblock zum QoS-Profile (Zeile 90-113).

**Fragen**:
1. Was passiert, wenn YOLO langsamer ist als die Bildrate (30 FPS)?
2. Was ist der Unterschied zwischen `depth=1` und `depth=10`?
3. Welchen Effekt hat `depth=1` auf die Anzahl der verarbeiteten Frames?

**Experiment**: Führen Sie den Detector mit beiden Einstellungen aus und vergleichen Sie:
- Anzahl der verarbeiteten Frames
- Laufzeit

### Aufgabe 2.3 (Optional): Bounding Boxes zeichnen

Erweitern Sie die `save_debug_image()` Funktion, um Bounding Boxes um erkannte Personen zu zeichnen.

**Schritte**:
1. Für jede Box in `person_boxes`: Extrahiere die Koordinaten (`x1, y1, x2, y2`) aus `box['xyxy']`
2. Zeichne ein Rechteck mit `cv2.rectangle()`
3. Füge die Konfidenz als Text mit `cv2.putText()` hinzu

**Testen**: Starten Sie den Detector mit `save_images:=true`:
```bash
ros2 launch person_detector detector.launch.py save_images:=true
```

Die Bilder werden in `results/debug_images/` gespeichert.

### Testen

```bash
# Paket neu bauen (nach Änderungen)
colcon build --packages-select person_detector
source install/setup.bash

# Terminal 1: Bag abspielen
ros2 bag play data/20251126_ifi2

# Terminal 2: YOLO-Detector starten
ros2 launch person_detector detector.launch.py

# Ctrl+C zum Beenden
```

**Erwartete Ausgabe**:
```
[INFO] Frame   10 | YOLO: 2 Personen | 142ms (7.0 FPS)
[INFO] Frame   20 | YOLO: 1 Personen | 138ms (7.2 FPS)
...
============================================================
YOLO DETECTION SUMMARY
============================================================
Total frames analyzed: 132
Effective FPS: 6.7
```

---

## Aufgabe 3: Analyse-Skript (30 min)

**Datei**: `analyze_results.py`

### Aufgabe 3.1: CSV laden

Vervollständigen Sie `load_csv()`:
- Öffnen Sie die CSV-Datei und lesen Sie sie mit dem `csv`-Modul
- Extrahieren Sie `timestamp` (float) und `count` (int) aus jeder Zeile
- Geben Sie die Daten sortiert nach Timestamp zurück

**Tipp**: Nutzen Sie `csv.DictReader` für einfachen Zugriff auf Spalten.

### Aufgabe 3.2: Timestamp-Matching

Vervollständigen Sie `match_timestamps()`:
- YOLO hat weniger Frames als ZED (wegen langsamer Inferenz)
- Für jeden YOLO-Datenpunkt: Finde den ZED-Datenpunkt mit dem nächsten Timestamp
- Toleranz: 100ms (0.1 Sekunden) - ignoriere Matches außerhalb dieser Toleranz

**Tipp**: Berechne die absolute Zeitdifferenz und finde das Minimum.

### Aufgabe 3.3: Plot erstellen

Vervollständigen Sie `create_plot()`:
- Erstellen Sie einen Step-Plot für beide Datenreihen
- YOLO: blaue durchgezogene Linie
- ZED: rote gestrichelte Linie
- Achsenbeschriftung, Legende und Titel nicht vergessen

**Tipp**: `ax.step()` eignet sich gut für diskrete Zähldaten.

### Testen

```bash
python3 analyze_results.py
```

**Erwartete Ausgabe**:
```
YOLO: 132 Datenpunkte geladen
ZED:  1305 Datenpunkte geladen
Gematchte Datenpunkte: 132
```

**Ergebnis**: `results/comparison_plot.png`

---

## Verzeichnisstruktur

```
exercise_02_person_detection/
├── data/                          → Symlink zum Datensatz
├── results/
│   ├── yolo_detections.csv        (generiert)
│   ├── zed_detections.csv         (generiert)
│   └── comparison_plot.png        (generiert)
├── analyze_results.py             ← Aufgabe 3
└── person_detector/               ← ROS 2-Paket
    ├── package.xml
    ├── setup.py
    ├── person_detector/
    │   ├── detector_node.py       ← Aufgabe 2
    │   └── zed_extractor_node.py  (fertig)
    └── launch/
        ├── detector.launch.py     (fertig)
        └── zed_extractor.launch.py ← Aufgabe 1
```

---

## Workflow-Zusammenfassung

```bash
# 1. Vorbereitung
cd exercises/exercise_02_person_detection
ln -s ../exercise_01_gnss_bags/data data

# 2. ROS 2 Paket bauen
colcon build --packages-select person_detector
source install/setup.bash

# 3. ZED-Daten extrahieren (2 Terminals)
ros2 bag play data/20251126_ifi2                    # Terminal 1
ros2 launch person_detector zed_extractor.launch.py # Terminal 2, Ctrl+C

# 4. YOLO-Daten extrahieren (2 Terminals)
ros2 bag play data/20251126_ifi2                    # Terminal 1
ros2 launch person_detector detector.launch.py     # Terminal 2, Ctrl+C

# 5. Analyse durchführen
python3 analyze_results.py

# 6. Ergebnisse prüfen
ls results/
```

---

## Hilfe und Ressourcen

- [YOLOv8 Dokumentation](https://docs.ultralytics.com/)
- [COCO Klassen](https://docs.ultralytics.com/datasets/detect/coco/) – Person = Klasse 0
- [cv_bridge](https://docs.ros.org/en/jazzy/p/cv_bridge/)
- [ROS 2 Launch](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)

---

## Abgabe

**Checkliste**:

- [ ] `zed_extractor.launch.py` funktioniert
- [ ] `detector_node.py` zählt Personen korrekt
- [ ] `analyze_results.py` erstellt Plot
- [ ] `results/comparison_plot.png` vorhanden
- [ ] Fragen zu QoS beantwortet
