# Übung 3: Linienverfolgung mit Hinderniserkennung

**Zeitaufwand**: 180 Minuten

**Voraussetzungen**: Vorlesungen 1-10, Übungen 1-2

**Gruppenarbeit**: 2-3 Personen

**Abgabe**: [Termin wird bekanntgegeben]

## Szenario: Autonomer Transportroboter

**Willkommen zurück bei RoboTech Solutions GmbH!**

Nach Ihren erfolgreichen Arbeiten an der GNSS-Analyse und Personendetektion hat das Logistikteam ein neues Projekt: Autonome Transportroboter sollen in der Fertigung Bauteile zwischen Arbeitsstationen transportieren. Die Roboter folgen dabei Bodenmarkierungen und müssen bei Hindernissen (z.B. abgestellte Kisten, Personen) sicher anhalten.

**Ihr Auftrag**: Entwickeln Sie im Team einen Prototyp auf Basis des TurtleBot 3, der:

1. Einer weißen Bodenlinie autonom folgt
2. Hindernisse auf der Fahrbahn erkennt und anhält
3. Nach Entfernung des Hindernisses selbstständig weiterfährt

---

## Teamorganisation (Pflicht!)

> **Wichtig**: Bevor Sie mit der technischen Arbeit beginnen, teilen Sie die Rollen in Ihrer Gruppe auf und tragen Sie diese im Protokoll ein!

| Rolle | Verantwortung | Aufgabe | Schwerpunkt |
|-------|---------------|---------|-------------|
| **Vision Engineer** | Linienerkennung mit Kamera | B.1 | OpenCV, Bildverarbeitung |
| **Control Engineer** | Regelung der Roboterbewegung | B.2 | PID-Regler, Kinematik |
| **Safety Engineer** | Hinderniserkennung & Sicherheit | B.3 | LiDAR, Zustandsautomat |

**Bei 2er-Gruppen** kombinieren Sie:
- Vision + Control (eine Person), Safety (eine Person), oder
- Vision (eine Person), Control + Safety (eine Person)

**Gemeinsame Aufgaben** (alle Teammitglieder):
- Teil A: Verbindung & Exploration
- Teil C: Integration, Test und Dokumentation

**So funktioniert die Zusammenarbeit**:
1. **Teil A** bearbeiten Sie gemeinsam am Roboter
2. **Teil B** kann parallel bearbeitet werden (jeder seine Aufgabe)
3. **Teil C** führen Sie die Komponenten zusammen und testen das Gesamtsystem

> **Tipp**: Kommunizieren Sie regelmäßig! Die Nodes müssen am Ende zusammenarbeiten. Einigen Sie sich früh auf Topic-Namen und Datenformate.

## Lernziele

Nach Abschluss dieser Übung können Sie:

- OpenCV für Echtzeit-Bildverarbeitung in ROS2 einsetzen
- Einen PID-Regler implementieren und tunen (Anwendung von VL 10)
- LiDAR-Daten für Hinderniserkennung auswerten
- Mehrere ROS2-Nodes zu einem Gesamtsystem integrieren
- Im Team an einem Robotikprojekt arbeiten

## Hardware

**TurtleBot 3 Burger**

| Komponente | Spezifikation |
|------------|---------------|
| LiDAR | RPLIDAR C1 (360°, 12m Reichweite, 10Hz) |
| Kamera | Raspberry Pi Camera v2 |
| Antrieb | Differential Drive (DYNAMIXEL) |
| Max. Geschwindigkeit | 0.22 m/s |
| Computer | Raspberry Pi 3 Model B Plus Rev 1.3 |

**Parcours**: Weiße Linie (Klebeband) auf dunklem Untergrund

**Netzwerk**: Ihr Laptop verbindet sich direkt mit dem TurtleBot (eigenes WLAN-Netz pro Gruppe)

## Architektur

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Ihr Laptop                                  │
│                                                                     │
│  ┌─────────────────┐    ┌─────────────────┐    ┌──────────────────┐ │
│  │ line_detector   │    │ controller_node │    │ obstacle_detector│ │
│  │                 │    │                 │    │                  │ │
│  │ Kamerabild      │    │ PID-Regler      │    │ LiDAR-Scan       │ │
│  │ → Linienpos.    │    │ → cmd_vel       │    │ → Hindernis?     │ │
│  └─────────────────┘    └────────┬────────┘    └──────────────────┘ │
│           ▲                      │                      ▲           │
│           │ /line_position       │ /cmd_vel             │ /obstacle │
│           │ (Float32)            │ (Twist)              │ (Bool)    │
│           │                      │                      │           │
└───────────┼──────────────────────┼──────────────────────┼───────────┘
            │                      │                      │
            │             ROS2 Netzwerk (DDS)             │
            │                      │                      │
┌───────────┼──────────────────────┼──────────────────────┼──────────┐
│           │                      ▼                      │          │
│  ┌────────┴────────┐    ┌─────────────────┐     ┌───────┴────────┐ │
│  │ /camera/        │    │ Motorsteuerung  │     │ /scan          │ │
│  │ image_raw       │    │                 │     │ (LaserScan)    │ │
│  └─────────────────┘    └─────────────────┘     └────────────────┘ │
│                                                                    │
│                         TurtleBot 3 Burger                         │
└────────────────────────────────────────────────────────────────────┘
```

## Vorbereitung

### ROS2 auf Ihrem Laptop

```bash
# ROS2 Jazzy sollte installiert sein
source /opt/ros/jazzy/setup.bash

# Benötigte Pakete prüfen
ros2 pkg list | grep -E "cv_bridge|image_transport"

# Falls nicht vorhanden:
sudo apt update
sudo apt install ros-jazzy-cv-bridge ros-jazzy-image-transport

# OpenCV für Python
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
pip install opencv-python numpy
```

### Netzwerkverbindung zum TurtleBot

Jede Gruppe erhält:
- WLAN-Name und Passwort für ihr Roboter-Netzwerk
- ROS_DOMAIN_ID (eindeutig pro Gruppe, notiert auf Roboter)

```bash
# 1. Mit dem Roboter-WLAN verbinden (siehe Zugangsdaten)

# 2. In jedem Terminal ROS_DOMAIN_ID setzen (verhindert Interferenz zwischen Gruppen)
export ROS_DOMAIN_ID=<IHRE_GRUPPEN_ID>

# 3. Verbindung testen
ros2 topic list

# Erwartete Topics vom TurtleBot:
# /camera/image_raw
# /scan
# /cmd_vel
# /odom
# ...
# Wenn Verbindung fehlgeschlagen:
# /parameter_events
# /rosout
```

### Workspace einrichten

```bash
# In diesem Übungsverzeichnis arbeiten
cd exercises/exercise_03_line_following

# ROS2 Paket bauen
colcon build --packages-select line_follower
source install/setup.bash
```

## Relevante Topics

| Topic | Message Type | Richtung | Beschreibung |
|-------|--------------|----------|--------------|
| `/camera/image_raw` | `sensor_msgs/Image` | TurtleBot → Laptop | RGB-Kamerabild |
| `/scan` | `sensor_msgs/LaserScan` | TurtleBot → Laptop | 360° LiDAR-Scan |
| `/cmd_vel` | `geometry_msgs/Twist` | Laptop → TurtleBot | Geschwindigkeitsbefehle |
| `/line_position` | `std_msgs/Float32` | Intern | Linienposition (-1.0 bis 1.0) |
| `/obstacle_detected` | `std_msgs/Bool` | Intern | Hindernis erkannt? |

---

## Teil A: Verbindung & Exploration (30 min)

> **Ziel**: Verbindung zum TurtleBot herstellen und Sensordaten verstehen
>
> **Bearbeitung**: Gemeinsam als Team

### Aufgabe A.0: Teamrollen festlegen (5 min)

**Bevor Sie beginnen:**

1. Legen Sie fest, wer welche Rolle übernimmt
2. Tragen Sie die Namen im Protokoll (`results/PROTOKOLL.md`) ein
3. Jedes Teammitglied liest die Aufgabe seiner Rolle (B.1, B.2 oder B.3) kurz durch

| Rolle | Name |
|-------|------|
| Vision Engineer (B.1) | _______________ |
| Control Engineer (B.2) | _______________ |
| Safety Engineer (B.3) | _______________ |

### Aufgabe A.1: Netzwerk einrichten

1. Verbinden Sie sich mit dem WLAN Ihres TurtleBots
2. Setzen Sie die `ROS_DOMAIN_ID` (wird vom Betreuer mitgeteilt)
3. Verifizieren Sie die Verbindung:

```bash
# Topics vom TurtleBot sehen?
ros2 topic list

# Kamerabild empfangen?
ros2 topic hz /camera/image_raw

# LiDAR-Daten empfangen?
ros2 topic hz /scan
```

### Aufgabe A.2: Sensordaten visualisieren

**Kamerabild anzeigen**:

```bash
# Option 1: rqt_image_view
ros2 run rqt_image_view rqt_image_view

# Option 2: Einfaches Python-Skript
python3 templates/view_camera.py
```

**LiDAR-Daten in RViz2**:

```bash
rviz2
# Add → By topic → /scan → LaserScan
# Fixed Frame: base_link oder base_scan

# Falls keine Anzeige:
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_scan laser
```

**Fragen zur Dokumentation**:

1. Welche Auflösung hat das Kamerabild?
2. Wie viele Punkte enthält ein LiDAR-Scan? In welchem Winkelbereich?
3. Können Sie die weiße Linie im Kamerabild erkennen?

### Aufgabe A.3: Erste Bewegungstests

Testen Sie die Teleop-Steuerung:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

**Wichtig**:
- Maximale Geschwindigkeit: 0.15 m/s (für diese Übung)
- Vorsichtig fahren, Hindernisse beachten!

**Fragen**:

1. Welche Taste fährt vorwärts? Welche dreht?
2. Was passiert bei `linear.x = 0.1` und `angular.z = 0.5`?

---

## Teil B: Komponenten entwickeln (120 min)

> **Ziel**: Drei ROS2-Nodes entwickeln, die zusammen das Linienfolge-System bilden

### Aufgabe B.1: Linienerkennung (45 min)

**Datei**: `line_follower/line_follower/line_detector_node.py`

**Template**: `templates/line_detector_template.py`

Der Linienerkennungs-Node soll:

1. Kamerabilder von `/camera/image_raw` empfangen
2. Die weiße Linie im Bild finden
3. Die horizontale Position der Linie publizieren

**Algorithmus**:

```
1. Bild zu OpenCV-Format konvertieren (cv_bridge)
2. Nur unteren Bildbereich betrachten (Region of Interest)
3. In Graustufen konvertieren
4. Schwellwert anwenden (weiß = Linie)
5. Schwerpunkt der weißen Pixel berechnen
6. Position normalisieren: -1.0 (links) bis +1.0 (rechts)
```

**Hilfreiche OpenCV-Funktionen**:

```python
# Graustufen
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Schwellwert (weiße Linie auf dunklem Grund)
_, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

# Schwerpunkt berechnen
moments = cv2.moments(binary)
if moments['m00'] > 0:
    cx = moments['m10'] / moments['m00']  # x-Koordinate des Schwerpunkts
```

**TODO im Template**:

1. `image_callback()`: Bild verarbeiten und Linienposition berechnen
2. `normalize_position()`: Pixelposition zu -1.0...+1.0 konvertieren
3. Debugging: Optional das verarbeitete Bild publizieren

**Testen**:

```bash
# Terminal 1: Node starten
ros2 run line_follower line_detector_node

# Terminal 2: Position beobachten
ros2 topic echo /line_position

# Erwartung: Wert ändert sich, wenn Roboter relativ zur Linie bewegt wird
```

### Aufgabe B.2: PID-Regler (45 min)

**Datei**: `line_follower/line_follower/controller_node.py`

**Template**: `templates/pid_controller_template.py`

Der Controller-Node soll:

1. Die Linienposition von `/line_position` empfangen
2. Den Fehler (Abweichung von der Mitte = 0) berechnen
3. Mit einem PID-Regler die Winkelgeschwindigkeit berechnen
4. `/cmd_vel` publizieren (wenn kein Hindernis)

**PID-Regler aus VL 10**:

$$u[k] = K_P \cdot e[k] + K_I \cdot \sum_{i=0}^{k} e[i] \cdot \Delta t + K_D \cdot \frac{e[k] - e[k-1]}{\Delta t}$$

Dabei ist:
- $e[k]$ = Fehler (Linienposition, Sollwert = 0)
- $u[k]$ = Stellgröße (Winkelgeschwindigkeit $\omega$)
- $K_P, K_I, K_D$ = Reglerparameter

**Empfohlene Startwerte**:

| Parameter | Startwert | Wirkung |
|-----------|-----------|---------|
| $K_P$ | 0.5 | Grundreaktion auf Fehler |
| $K_I$ | 0.0 | Erstmal deaktivieren |
| $K_D$ | 0.1 | Dämpfung |
| $v$ (linear) | 0.1 m/s | Konstante Vorwärtsgeschwindigkeit |

**TODO im Template**:

1. `pid_update()`: PID-Berechnung implementieren
2. `line_position_callback()`: Fehler berechnen, PID aufrufen
3. `publish_cmd_vel()`: Twist-Message erstellen und senden
4. Parameter über `config/params.yaml` konfigurierbar machen

**Testen (ohne Hindernis-Check)**:

```bash
# Terminal 1: Linienerkennung
ros2 run line_follower line_detector_node

# Terminal 2: Controller
ros2 run line_follower controller_node

# Beobachten: Roboter sollte der Linie folgen (evtl. oszillierend)
```

**PID-Tuning dokumentieren**:

Testen Sie verschiedene Parameter und dokumentieren Sie:

| $K_P$ | $K_I$ | $K_D$ | Beobachtung |
|-------|-------|-------|-------------|
| 0.5 | 0.0 | 0.0 | ? |
| 1.0 | 0.0 | 0.0 | ? |
| 0.5 | 0.0 | 0.1 | ? |
| ... | ... | ... | ... |

### Aufgabe B.3: Hinderniserkennung (30 min)

**Datei**: `line_follower/line_follower/obstacle_detector_node.py`

**Template**: `templates/obstacle_detector_template.py`

Der Obstacle-Detector-Node soll:

1. LiDAR-Daten von `/scan` empfangen
2. Prüfen, ob ein Hindernis im Fahrbereich ist
3. `/obstacle_detected` (Bool) publizieren

**LaserScan-Message verstehen**:

```python
# sensor_msgs/LaserScan
msg.ranges[]        # Array mit Distanzen (in Metern)
msg.angle_min       # Startwinkel (meist -π)
msg.angle_max       # Endwinkel (meist +π)
msg.angle_increment # Winkel zwischen Messungen
```

**Fahrbereich definieren**:

```
        Hindernis?
           │
    ┌──────┼──────┐
    │      │      │
    │   ┌──▼──┐   │   Prüfbereich:
    │   │     │   │   - Winkel: -30° bis +30° (vor dem Roboter)
    │   │ ROI │   │   - Distanz: < 0.35m = Hindernis
    │   │     │   │
    └───┴─────┴───┘
         ▲
      Roboter
```

**TODO im Template**:

1. `scan_callback()`: Relevante Winkel filtern
2. `check_obstacle()`: Minimale Distanz im ROI prüfen
3. Schwellwert als Parameter konfigurierbar

**Testen**:

```bash
# Terminal 1: Obstacle Detector
ros2 run line_follower obstacle_detector_node

# Terminal 2: Status beobachten
ros2 topic echo /obstacle_detected

# Test: Hand vor den Roboter halten → True
```

---

## Teil C: Integration & Dokumentation (30 min)

### Aufgabe C.1: Launch-File erstellen

**Datei**: `line_follower/launch/line_following.launch.py`

Erstellen Sie ein Launch-File, das alle drei Nodes startet:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='line_follower',
            executable='line_detector_node',
            name='line_detector',
            # parameters=[...]
        ),
        Node(
            package='line_follower',
            executable='obstacle_detector_node',
            name='obstacle_detector',
        ),
        Node(
            package='line_follower',
            executable='controller_node',
            name='controller',
            # parameters=[...]
        ),
    ])
```

**Testen des Gesamtsystems**:

```bash
# Paket neu bauen
colcon build --packages-select line_follower
source install/setup.bash

# Alle Nodes starten
ros2 launch line_follower line_following.launch.py

# Test-Szenario:
# 1. Roboter auf die Linie setzen → folgt der Linie
# 2. Hindernis auf die Linie stellen → Roboter stoppt
# 3. Hindernis entfernen → Roboter fährt weiter
```

### Aufgabe C.2: Systemtest & Video

Führen Sie einen vollständigen Testlauf durch:

1. Roboter am Start der Linie positionieren
2. System starten
3. Roboter folgt der Linie
4. Hindernis platzieren → Stopp
5. Hindernis entfernen → Weiterfahrt
6. **Video aufnehmen** (Handy genügt, ~30 Sekunden)

### Aufgabe C.3: Protokoll erstellen

**Datei**: `results/PROTOKOLL.md`

```markdown
# Protokoll: Linienverfolgung mit Hinderniserkennung

**Datum**: [Datum]
**Gruppe**: [Namen der Teammitglieder]
**Rollenverteilung**:
- Vision Engineer: [Name]
- Control Engineer: [Name]
- Safety Engineer: [Name]

## 1. Systemübersicht

### 1.1 Architekturdiagramm
[Skizze oder Beschreibung der Node-Kommunikation]

### 1.2 Verwendete Topics
| Topic | Publisher | Subscriber | Frequenz |
|-------|-----------|------------|----------|
| ... | ... | ... | ... Hz |

## 2. Linienerkennung

### 2.1 Algorithmus
[Kurze Beschreibung: Welchen Ansatz haben Sie gewählt?]

### 2.2 Parameter
- ROI (Region of Interest): [Bildbereich]
- Schwellwert: [Wert]

### 2.3 Herausforderungen
[Was hat gut funktioniert? Was war schwierig?]

## 3. PID-Regelung

### 3.1 Finale Parameter
| Parameter | Wert |
|-----------|------|
| $K_P$ | |
| $K_I$ | |
| $K_D$ | |
| $v$ (linear) | |

### 3.2 Tuning-Prozess
[Welche Werte haben Sie getestet? Was war die Auswirkung?]

| $K_P$ | $K_I$ | $K_D$ | Beobachtung |
|-------|-------|-------|-------------|
| | | | |

### 3.3 Bezug zu VL 10
[Wie haben sich die theoretischen Konzepte in der Praxis bestätigt?]

## 4. Hinderniserkennung

### 4.1 Parameter
- Prüfwinkel: [z.B. -30° bis +30°]
- Stopp-Distanz: [z.B. 0.35 m]

### 4.2 Zuverlässigkeit
[Wie zuverlässig wurde das Hindernis erkannt?]

## 5. Gesamtsystem

### 5.1 Testlauf
- [ ] Linie erkannt
- [ ] Roboter folgt Linie stabil
- [ ] Hindernis erkannt → Stopp
- [ ] Nach Entfernung → Weiterfahrt

### 5.2 Video
[Link oder Dateiname des Videos]

## 6. Fazit

### 6.1 Was haben wir gelernt?
[Wichtigste Erkenntnisse der Übung]

### 6.2 Verbesserungsvorschläge
[Was würden Sie bei mehr Zeit verbessern?]

---

**Erstellungswerkzeuge**: ROS2 Jazzy, Python 3, OpenCV
**Hardware**: TurtleBot 3 Burger
```

---

## Verzeichnisstruktur

```
exercise_03_line_following/
├── README.md                          ← Diese Datei
├── templates/
│   ├── line_detector_template.py      ← Vorlage Linienerkennung
│   ├── pid_controller_template.py     ← Vorlage PID-Regler
│   ├── obstacle_detector_template.py  ← Vorlage Hinderniserkennung
│   └── view_camera.py                 ← Hilfsskript Kameraansicht
├── line_follower/                     ← ROS2-Paket
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/
│   │   └── line_follower
│   ├── line_follower/
│   │   ├── __init__.py
│   │   ├── line_detector_node.py      ← Aufgabe B.1
│   │   ├── controller_node.py         ← Aufgabe B.2
│   │   └── obstacle_detector_node.py  ← Aufgabe B.3
│   ├── launch/
│   │   └── line_following.launch.py   ← Aufgabe C.1
│   └── config/
│       └── params.yaml
└── results/
    ├── PROTOKOLL.md                   ← Hauptabgabe
    ├── pid_tuning.csv                 ← PID-Experimente
    └── demo_video.mp4                 ← Testvideo
```

---

## Abgabe

**Deadline**: [wird bekanntgegeben]

**Checkliste**:

- [ ] `line_detector_node.py` erkennt weiße Linie
- [ ] `controller_node.py` implementiert PID-Regler
- [ ] `obstacle_detector_node.py` erkennt Hindernisse
- [ ] `line_following.launch.py` startet alle Nodes
- [ ] `PROTOKOLL.md` vollständig ausgefüllt
- [ ] Video des funktionierenden Systems

**Format**:

```bash
cd exercises
tar -czf exercise_03_GRUPPENNAME.tar.gz exercise_03_line_following/
```

---

## Tipps & Fehlerbehebung

### Kein Kamerabild?

```bash
# Topic vorhanden?
ros2 topic list | grep camera

# Daten kommen an?
ros2 topic hz /camera/image_raw

# Mögliche Ursachen:
# - ROS_DOMAIN_ID nicht gesetzt
# - WLAN-Verbindung unterbrochen
# - Kamera auf TurtleBot nicht gestartet
```

### Roboter reagiert nicht auf /cmd_vel?

```bash
# Manuell testen
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -1

# Achtung: Evtl. anderer Node blockiert /cmd_vel (z.B. teleop)
```

### PID oszilliert stark?

- $K_P$ zu hoch → reduzieren
- $K_D$ erhöhen für Dämpfung
- Langsamere Geschwindigkeit ($v$) verwenden

### Linie wird nicht erkannt?

- Schwellwert anpassen (Lichtverhältnisse!)
- ROI prüfen (sieht die Kamera die Linie?)
- Debug-Bild anzeigen lassen

---

## Hilfe und Ressourcen

**ROS2 Dokumentation**:
- [cv_bridge Tutorial](https://docs.ros.org/en/jazzy/p/cv_bridge/)
- [LaserScan Message](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html)
- [Twist Message](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)

**TurtleBot 3**:
- [TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [TurtleBot3 ROS2](https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_setup/)

**OpenCV**:
- [Thresholding Tutorial](https://docs.opencv.org/4.x/d7/d4d/tutorial_py_thresholding.html)
- [Image Moments](https://docs.opencv.org/4.x/d8/d23/classcv_1_1Moments.html)

**Vorlesungsmaterial**:
- VL 10: Kinematik und Regelung (PID-Regler, Differentialantrieb)
- VL 6: Kamera & Stereo (Bildverarbeitung)
- VL 7: Objekterkennung (OpenCV-Grundlagen)

---

**Viel Erfolg!**
