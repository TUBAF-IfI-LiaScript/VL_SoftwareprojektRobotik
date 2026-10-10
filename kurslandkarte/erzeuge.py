"""Kurslandkarte Robotik: Daten und Seite erzeugen.

Abschnitte (Foliennummern) werden automatisch aus den Materialien gelesen,
Termine, Begriffe, Beziehungen und didaktische Fäden sind unten kuratiert.

Aufruf aus dem Repository-Wurzelverzeichnis:

    python3 kurslandkarte/erzeuge.py

Schreibt docs/kurslandkarte/kurslandkarte.json und docs/kurslandkarte/index.html.

Autoren: Sebastian Zug & Claude Code (Opus 5.5) mit LiaScript Teaching-Agent
"""
import datetime, json, re, os

HIER = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HIER)
ZIEL = os.path.join(REPO, "docs", "kurslandkarte")
STAND = datetime.date.today().strftime("%d.%m.%Y")

KURS = {
    "basis": "https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/master/",
    "speicher": "robotik-map",
    "autoren": "Sebastian Zug & Claude Code (Opus 5.5) mit LiaScript Teaching-Agent",
}

# ---- Termine nach dem vorläufigen Zeitplan (Zeitplan.ods, Ansatz 1; Datum = Montag der KW)
# key, Bezeichnung, Datum, Kurztitel, Datei (None = noch kein Material), Status, Block
# Status: "ausgearbeitet" (neu für 2026/27), "Bestand" (Material aus 2025/26), "geplant"
T = [
    ("00",  "VL",    "2026-10-19", "Einführung",          "00_Einfuehrung/00_Einfuehrung.md",                     "ausgearbeitet", 1),
    ("ue1", "Ü 1",   "2026-10-26", "Linux Shell",         "exercises/exercise_00_linux_shell/README.md",          "ausgearbeitet", 1),
    ("03",  "VL",    "2026-10-26", "ROS 2 Grundlagen",    "03_ROS_Grundlagen_I/03_EinfuehrungROS.md",             "ausgearbeitet", 1),
    ("04",  "VL",    "2026-11-02", "ROS 2 Pakete",        "04_ROS_Pakete_Tools/04_ROS_Pakete.md",                 "ausgearbeitet", 1),
    ("ue2", "Ü 2",   "2026-11-09", "Installation ROS 2",  None, "geplant", 1),
    ("02",  "VL",    "2026-11-09", "Koordinaten + QoS",   "02_Koordinatensysteme/02_Koordinatentransformation.md", "Bestand", 1),
    ("p1",  "P 1",   "2026-11-23", "ROS 2 + Bags",        None, "geplant", 1),
    ("01",  "VL",    "2026-11-30", "Sensorik",            "01_Sensorik/01_Sensorik.md",                           "Bestand", 2),
    ("05",  "VL",    "2026-12-07", "Bildverarbeitung",    "05_Bildverarbeitung/05_Bildverarbeitung.md",           "Bestand", 2),
    ("06",  "VL",    "2026-12-07", "Kamera",              "06_Kamera_Stereo/06_Kamera_Stereo.md",                 "Bestand", 2),
    ("lid", "VL",    "2026-12-14", "LiDAR",               None, "geplant", 2),
    ("ue3", "Ü 3",   "2027-01-04", "Datenvisualisierung", None, "geplant", 2),
    ("ue4", "Ü 4",   "2027-01-11", "Kameradaten",         None, "geplant", 2),
    ("p2",  "P 2",   "2027-01-11", "Bildkorrektur",       None, "geplant", 2),
    ("p3",  "P 3",   "2027-01-18", "Spurerkennung",       None, "geplant", 2),
    ("07",  "VL",    "2027-01-25", "Objekterkennung",     "07_Objekterkennung/07_Objekterkennung.md",             "Bestand", 3),
    ("pkw", "VL",    "2027-01-25", "… in Punktwolken",    None, "geplant", 3),
    ("ue5", "Ü 5",   "2027-02-01", "Objekte im LiDAR",    None, "geplant", 3),
    ("p4",  "P 4",   "2027-02-08", "Hinderniserkennung",  None, "geplant", 3),
    ("ue6", "Ü 6",   "2027-04-05", "YOLO vs. klassisch",  None, "geplant", 3),
    ("08",  "VL",    "2027-04-12", "Sensorfusion I",      "08_Sensorfusion_I/08_Datenfusion.md",                  "Bestand", 4),
    ("09",  "VL",    "2027-04-19", "Sensorfusion II",     "09_Sensorfusion_II/09_Sensorfusion_II.md",             "Bestand", 4),
    ("ue7", "Ü 7",   "2027-04-19", "2D-EKF",              None, "geplant", 4),
    ("p5",  "P 5",   "2027-04-26", "Streckenverlauf",     None, "geplant", 4),
    ("10",  "VL",    "2027-05-03", "Kinematik",           "10_Kinematik_Regelung/10_Kinematik_Regelung.md",       "Bestand", 5),
    ("ue8", "Ü 8",   "2027-05-03", "Reglervergleich",     None, "geplant", 5),
    ("11",  "VL",    "2027-05-10", "Navigation",          "11_Navigation/11_Navigation.md",                       "Bestand", 5),
    ("slm", "VL",    "2027-05-17", "SLAM",                None, "geplant", 5),
    ("ue9", "Ü 9",   "2027-05-17", "Kartenerstellung",    None, "geplant", 5),
    ("p6",  "P 6",   "2027-05-24", "Mapping + Bahnfolge", None, "geplant", 5),
    ("p7",  "P 7",   "2027-05-31", "Parken + Abbiegen",   None, "geplant", 6),
    ("p8",  "P 8",   "2027-06-07", "Praktikum + X",       None, "geplant", 6),
]
BLOECKE = {1: "Block 1 · ROS-Grundlagen", 2: "Block 2 · Sensorgrundlagen", 3: "Block 3 · Objekterkennung",
           4: "Block 4 · Sensorfusion", 5: "Block 5 · Regelung & Navigation", 6: "Block 6 · Praktikum + X"}
BLOCKFARBE = {1: "--phase-einstieg", 2: "--phase-grundlagen", 3: "--phase-analyse",
              4: "--phase-einstieg", 5: "--phase-grundlagen", 6: "--phase-zaesur"}
LINIEN = [
    ("werkzeuge", "Linux & Werkzeuge"), ("ros", "ROS 2"), ("mathe", "Koordinaten & Modelle"),
    ("wahrnehmung", "Sensorik & Wahrnehmung"), ("schaetzung", "Fusion & Schätzung"),
    ("bewegung", "Regelung & Navigation"), ("projekt", "Projektaufgabe"),
]
assert [t[2] for t in T] == sorted(t[2] for t in T), "Termine nicht chronologisch"
# Vorlesungen laufend nach Zeitplan nummerieren (VL 1, VL 2, …); die Ordnernummern
# im Repo (03_…, 02_…) stammen aus der alten Reihenfolge und erscheinen nicht.
_nr = 0
for _i, _t in enumerate(T):
    if _t[1] == "VL":
        _nr += 1
        T[_i] = (_t[0], f"VL {_nr}") + _t[2:]
IDX = {t[0]: i for i, t in enumerate(T)}

# ---- Abschnitte automatisch (LiaScript-Foliennummer = Position der Überschrift)
def abschnitte(datei):
    s = open(os.path.join(REPO, datei), encoding="utf-8").read()
    s = s[s.find("-->") + 3:]
    s = re.sub(r"^(`{3,4}).*?\n.*?\n\1", "", s, flags=re.S | re.M)
    return [m.group(2).strip().replace("`", "") for m in re.finditer(r"^(#{1,6}) +(.+)$", s, re.M)]

sections = {t[0]: abschnitte(t[4]) if t[4] else [] for t in T}

# ---- Begriffe: id, Name, Linie, Termin-key, Abschnittstitel, Ebene, Code?, Erklärung, Fehlvorstellung
B = [
    # --- VL 00 Einführung
    ("verkehrswelt", "Projektaufgabe Verkehrswelt", "projekt", "00", 'Unser Ziel: Autonomes Fahren im Miniaturformat', 1, 0, "Ein TurtleBot fährt autonom auf schwarzem Untergrund: Spur folgen, Ampeln und Schilder beachten, Fußgängern ausweichen, einparken.", None),
    ("formate", "VL, Ü und P", "projekt", "00", 'Wie läuft die Veranstaltung ab?', 2, 0, "Vorlesung führt Konzepte ein, Übung festigt sie am eigenen Rechner, Praktikum setzt sie am TurtleBot um — je 90 Minuten.", None),
    ("roboter", "Roboter", "wahrnehmung", "00", 'Was ist ein Roboter?', 1, 0, "Es gibt keine einheitliche Definition; VDI, RIA und JRA betonen Programmierbarkeit, Bewegung und Sensorik unterschiedlich.", "„Ein Roboter ist eine Maschine mit Armen.“"),
    ("klassifikation", "Klassifikation", "wahrnehmung", "00", 'Klassifikation von Robotersystemen', 2, 0, "Autonomie, Mobilität und Komplexität der Sensorik sind unabhängige Kriterien.", "„Mobil heißt autonom.“"),
    ("herausforderungen", "Herausforderungen", "bewegung", "00", 'Herausforderungen bei der Entwicklung mobiler Roboter', 2, 0, "Positionierung, Kartierung, Hinderniserkennung und Echtzeit — dazu wirtschaftliche und rechtliche Fragen.", None),
    ("sta", "Sense-Think-Act", "bewegung", "00", 'Systemarchitektur: Das Sense-Think-Act-Paradigma', 1, 0, "Wahrnehmen, Entscheiden, Handeln — gegliedert in reaktive Überwachung, Ausführung und Handlungsplanung.", None),
    ("turtlebot", "TurtleBot 3 Burger", "projekt", "00", 'Sense-Think-Act am TurtleBot', 1, 0, "Unsere Plattform: RPLIDAR C1, Pi-Kamera, Raspberry Pi 3 B+, Differentialantrieb, maximal 0,22 m/s.", None),
    ("rosmotivation", "Warum ROS?", "ros", "00", 'Warum ROS? Das "Rad nicht neu erfinden"', 1, 0, "ROS 2 liefert Kommunikation, wiederverwendbare Pakete und Werkzeuge, damit nicht jedes Team alles neu baut.", "„ROS ist ein Betriebssystem.“"),

    # --- Ü 1 Linux Shell
    ("shell", "Terminal & Shell", "werkzeuge", "ue1", 'Warum Terminal?', 1, 0, "Das Terminal zeigt Text, die Shell (bash) interpretiert die Eingaben — der Weg zu ROS 2 und zum TurtleBot.", None),
    ("pfade", "Absolute & relative Pfade", "werkzeuge", "ue1", 'Dateisystem', 1, 0, "Absolute Pfade beginnen bei /, relative beim aktuellen Verzeichnis; . und .. stehen für hier und darüber.", None),
    ("pipes", "Pipes & Umleitung", "werkzeuge", "ue1", 'Dateien ansehen, suchen und verknüpfen', 2, 1, "| reicht die Ausgabe weiter, > und >> schreiben sie in eine Datei — z. B. grep ERROR ros.log | wc -l.", None),
    ("rechte", "Dateirechte", "werkzeuge", "ue1", 'Rechte und Benutzer', 2, 1, "rwx für Besitzer, Gruppe und andere; chmod +x macht ein Skript ausführbar, chmod 754 setzt Rechte als Zahl.", "„Ein Skript, das ich geschrieben habe, darf ich ausführen.“"),
    ("prozess", "Prozess", "werkzeuge", "ue1", 'Prozesse', 2, 0, "Jedes laufende Programm hat eine PID — jeder ROS-Knoten ist ein eigener Prozess.", None),
    ("umgebung", "Umgebungsvariable", "werkzeuge", "ue1", 'Umgebungsvariablen', 1, 0, "Werte wie PATH oder ROS_DOMAIN_ID, die eine Shell an gestartete Programme weitergibt — nur mit export.", "„Eine Variable gilt in allen Terminals.“"),
    ("path", "PATH", "werkzeuge", "ue1", 'PATH — wo findet die Shell Programme?', 2, 1, "Die Shell sucht Befehle in den Verzeichnissen aus PATH; fehlt ROS 2 dort: command not found.", None),
    ("source", "source", "werkzeuge", "ue1", 'source oder ausführen?', 1, 1, "Führt ein Skript in der aktuellen Shell aus, damit gesetzte Variablen erhalten bleiben — source /opt/ros/jazzy/setup.bash.", "„./setup.bash und source setup.bash sind dasselbe.“"),
    ("bashrc", "~/.bashrc", "werkzeuge", "ue1", 'Aufgabe 4: Zwei Terminals, zwei Welten', 2, 1, "Wird beim Öffnen jedes Terminals ausgeführt — der Ort für source und ROS_DOMAIN_ID.", None),
    ("ssh", "ssh", "werkzeuge", "ue1", 'Pakete installieren und Fernzugriff', 2, 1, "Anmelden auf einem entfernten Rechner, z. B. dem Raspberry Pi des TurtleBots.", None),
    # --- VL 01 Sensorik
    ("sensor", "Sensor", "wahrnehmung", "01", 'Was ist ein Sensor?', 1, 0, "Ein Sensor wandelt physikalische, chemische oder biologische Messgrößen in elektrische Signale um.", None),
    ("propextero", "Propriozeptiv/exterozeptiv", "wahrnehmung", "01", 'Klassifikation von Sensoren', 1, 0, "Propriozeptive Sensoren messen den Roboterzustand (Encoder, IMU), exterozeptive die Umgebung (Kamera, Lidar, Ultraschall).", None),
    ("sensorparam", "Sensorparameter", "wahrnehmung", "01", 'Wichtige Sensorparameter', 2, 0, "Messbereich, Auflösung, Linearität, Messfrequenz, Querempfindlichkeit und Öffnungswinkel charakterisieren einen Sensor.", "„Eine hohe Auflösung bedeutet automatisch eine hohe Genauigkeit.“"),
    ("pose", "Pose (6 DoF)", "mathe", "01", 'Propriozeptive Sensorik', 1, 0, "Die Pose kombiniert Position (x, y, z) und Orientierung (Rotationsmatrix oder Quaternion) zu sechs Freiheitsgraden.", None),
    ("imu", "IMU", "wahrnehmung", "01", 'Inertialsensorik (IMU)', 1, 0, "Eine IMU kombiniert Beschleunigungssensoren, Gyroskope und Magnetometer und misst je drei Achsen.", "„Das Magnetometer liefert auch in Innenräumen eine verlässliche Kursreferenz.“"),
    ("drift", "Drift durch Integration", "schaetzung", "01", 'Inertialsensorik (IMU)', 2, 0, "Integriert man Beschleunigung oder Drehrate auf, summieren sich Fehler zu Drift — Abhilfe schafft Fusion mit absoluten Sensoren.", "„Aus Beschleunigungsdaten lässt sich die Position dauerhaft genau aufintegrieren.“"),
    ("odometrie", "Odometrie", "wahrnehmung", "01", 'Odometrie', 1, 0, "Odometrie bestimmt die Bewegung aus Radumdrehungen, die Encoder photoelektrisch, magnetisch oder induktiv erfassen.", None),
    ("quadratur", "Zwei-Kanal-Encoder", "wahrnehmung", "01", 'Odometrie', 2, 0, "Zwei um 90° phasenverschobene Kanäle ermöglichen Drehrichtungserkennung und eine höhere Auflösung.", None),
    ("tof", "Laufzeitmessung", "wahrnehmung", "01", 'Laufzeitmessung (Ultraschall)', 1, 0, "Die Entfernung folgt aus der Echolaufzeit, d = ½·c·t; Mehrfachechos und Öffnungswinkel begrenzen die Messung.", "„Die Schallgeschwindigkeit ist konstant, Ultraschall misst daher temperaturunabhängig.“"),
    ("triangulation", "Triangulation", "wahrnehmung", "01", 'Triangulation', 2, 0, "Aus einer bekannten Basislinie zwischen Sender und Empfänger wird der Abstand geometrisch bestimmt, z. B. bei IR-Sensoren.", None),
    ("gnss", "GNSS-Positionsbestimmung", "wahrnehmung", "01", 'Mathematische Grundlagen der GNSS-Positionsbestimmung', 1, 0, "Pseudoranges zu mindestens vier Satelliten ergeben ein Gleichungssystem für Position und Empfänger-Uhrfehler.", "„Für eine 3D-Position genügen drei Satelliten.“"),
    ("dop", "GNSS-Fehler & DOP", "schaetzung", "01", 'Genauigkeit und Fehlerquellen bei GNSS', 2, 0, "Atmosphäre, Mehrwegeffekte und Satellitengeometrie (DOP) begrenzen die Genauigkeit; DGPS und SAPOS korrigieren.", None),
    # --- VL 02 Koordinaten
    ("frame", "Bezugskoordinatensystem", "mathe", "02", 'Mathematische Beschreibung', 1, 0, "Koordinaten eines Punktes haben nur zusammen mit dem Bezugssystem eine Bedeutung, das als Index angegeben wird (p_A).", None),
    ("starrtrafo", "Translation und Rotation", "mathe", "02", 'Relevante Transformationen', 1, 0, "Körpererhaltende Transformationen setzen sich aus Verschiebung (Vektoraddition) und Drehung (Rotationsmatrix) zusammen.", None),
    ("homtrafo", "Homogene Koordinaten", "mathe", "02", 'Homogene Koordinaten', 1, 0, "Eine zusätzliche Koordinate macht auch die Translation zu einer Matrixmultiplikation.", None),
    ("invtrafo", "Inverse Transformation", "mathe", "02", 'Inverse Transformation', 2, 0, "Die Umkehrung ist die entgegengesetzte Verschiebung bzw. Drehung um −φ (bei Rotation die Transponierte).", "„Für die Rücktransformation muss man die Matrix immer numerisch invertieren.“"),
    ("verkettung", "Verkettung", "mathe", "02", 'Kombination von Transformationen', 1, 0, "Transformationen entlang einer Frame-Kette (world → base → camera) werden multipliziert; die Reihenfolge ist entscheidend.", "„Ob man erst rotiert oder erst verschiebt, ist egal.“"),
    ("euler", "Euler-Winkel", "mathe", "02", 'Homogene Koordinaten in 3D', 2, 0, "Gieren, Nicken und Rollen beschreiben 3D-Rotationen anschaulich, leiden aber unter Gimbal Lock und Mehrdeutigkeit.", "„Jede Orientierung hat genau ein Euler-Winkel-Tripel.“"),
    ("quaternion", "Quaternion", "mathe", "02", 'Quaternionen', 1, 0, "Vier Werte beschreiben eine Drehung um eine Achse ohne Singularitäten — der Standard für Rotationen in ROS.", None),
    ("tf2", "tf2", "ros", "02", 'Umsetzung in ROS', 1, 0, "tf2 verwaltet die zeitabhängigen Transformationen zwischen Frames und rechnet Daten zwischen Koordinatensystemen um.", None),
    ("frameid", "Header & frame_id", "ros", "02", 'Umsetzung in ROS', 2, 0, "Jede Sensornachricht trägt einen Header mit Zeitstempel und frame_id, der sie einem Koordinatensystem zuordnet.", None),
    ("statictf", "Statische Transformation", "ros", "02", 'Beispiel 1 Laserscanner', 2, 1, "Konstante Relationen wie Sensor zu Roboter stellt z. B. static_transform_publisher bereit.", None),
    ("tftree", "tf-Baum", "ros", "02", 'Beispiel 2 - Turtle', 2, 0, "Alle Frames bilden einen Baum; die Transformation zwischen zwei Frames ergibt sich über den Pfad im Baum.", None),
    ("urdf", "URDF", "ros", "02", 'Beispiel 3 - URDF für statische Relationen', 2, 1, "XML-Beschreibung von Links, Joints, Geometrie und Sensoren eines Roboters — und damit seiner statischen Frames.", None),
    # --- VL 03 ROS 2 Grundlagen
    ("distro", "ROS-2-Distribution", "ros", "03", 'ROS, was ist das?', 2, 1, "Jährlich im Mai erscheint eine Distribution; wir nutzen die LTS-Version Jazzy Jalisco auf Ubuntu 24.04.", "„Beispiele mit roscore oder catkin funktionieren auch unter ROS 2.“"),
    ("middleware", "ROS als Middleware", "ros", "03", 'ROS, was ist das?', 1, 0, "ROS bietet Hardwareabstraktion, Treiber, Nachrichtenaustausch und Paketverwaltung für verteilte Robotersoftware.", None),
    ("dds", "DDS / rmw-Schicht", "ros", "03", 'Architektur von ROS 2', 1, 0, "ROS 2 trennt Client-Libraries (rclcpp/rclpy) über die rmw-Schicht von der austauschbaren Middleware — Standard ist Fast DDS, Alternativen sind Cyclone DDS oder Zenoh.", "„Knoten mit unterschiedlicher RMW_IMPLEMENTATION finden sich trotzdem zuverlässig.“"),
    ("node", "Knoten (Node)", "ros", "03", 'Basiskonzepte', 1, 0, "Ein Knoten ist ein Teilnehmer im ROS-Graphen, der Topics veröffentlicht oder abonniert.", None),
    ("topic", "Topic", "ros", "03", 'Basiskonzepte', 1, 0, "Topics sind benannte Kanäle, die Publisher und Subscriber entkoppeln (n:m-Kommunikation).", "„Publisher und Subscriber müssen einander kennen.“"),
    ("message", "Nachricht (Message)", "ros", "03", 'Basiskonzepte', 1, 0, "Eine Message legt Struktur und Datentypen der übertragenen Daten fest, z. B. sensor_msgs/msg/LaserScan.", None),
    ("msgdef", "Aufbau einer Nachricht", "ros", "03", 'Basiskonzepte', 2, 1, "ros2 interface show zeigt die Felder eines Nachrichtentyps — z. B. Twist mit linear.x (vorwärts) und angular.z (drehen).", "„Bei Twist ist linear.z die Vorwärtsgeschwindigkeit.“"),
    ("discovery", "Discovery", "ros", "03", 'Basiskonzepte', 2, 0, "Knoten finden sich automatisch innerhalb derselben ROS_DOMAIN_ID, verbinden sich aber nur bei kompatiblen QoS-Einstellungen.", "„Zwei Knoten auf demselben Topic kommunizieren immer.“"),
    ("kommmuster", "Service, Parameter, Action", "ros", "03", 'Basiskonzepte', 2, 0, "Neben Topics: Services für Anfrage/Antwort, Parameter zur Konfiguration, Actions für lang laufende Aufträge mit Rückmeldung.", "„Für jede Kommunikation nimmt man am besten ein Topic.“"),
    ("tbtopics", "Topics des TurtleBot", "ros", "03", 'Vom turtlesim zum TurtleBot', 1, 1, "Der TurtleBot stellt Sensoren und Antrieb als Topics bereit: /scan, /odom, /imu, Kamerabild und /cmd_vel.", "„Für den echten Roboter braucht man ganz andere Werkzeuge als für den turtlesim.“"),
    ("ros2cli", "ros2-Kommandozeile", "werkzeuge", "03", 'Einführungsbeispiele', 1, 1, "ros2 untersucht und steuert das System über Unterbefehle wie node, topic, service, param, bag und launch.", None),
    ("pubsub", "Publisher/Subscriber-API", "ros", "03", 'Hello-World Implementierung', 2, 1, "Knoten legen per create_publisher/create_subscription Kanäle an und laufen mit spin().", None),
    ("topiccli", "ros2 topic …", "werkzeuge", "03", 'Hello-World Implementierung', 2, 1, "Mit ros2 topic list/info/hz/echo/pub lassen sich Topics inspizieren und von Hand Nachrichten senden.", None),
    ("remap", "Remapping", "ros", "03", 'Hello-World Implementierung', 2, 1, "Mit --ros-args -r topic:=neu (oder __node:=name) werden Topic- und Knotennamen beim Start umgelenkt, ohne den Code zu ändern.", "„Der Topic-Name ist fest im Code verankert.“"),
    ("rqt", "rqt / rqt_graph", "werkzeuge", "03", 'turtlesim: ROS 2 zum Anfassen', 2, 1, "rqt bündelt die Inspektionsfunktionen der ros2-CLI grafisch, z. B. den Knoten-Topic-Graphen.", None),
    # --- VL 04 ROS 2 Pakete
    ("domainid", "ROS_DOMAIN_ID", "ros", "04", 'ROS 2 auf mehreren Rechnern', 2, 1, "Die Umgebungsvariable trennt ROS-2-Systeme im selben Netz — im Labor eine pro TurtleBot-Gruppe, gesetzt auf Laptop und Roboter.", "„Die ROS_DOMAIN_ID muss nur auf dem Laptop gesetzt werden.“"),
    ("paket", "ROS-2-Paket", "ros", "04", 'Vom Beispiel zum eigenen Paket', 1, 0, "Ein Paket kapselt Code samt Abhängigkeiten, Build-Konfiguration, Nachrichten und Launch-Files.", None),
    ("workspace", "Workspace", "werkzeuge", "04", 'Workspace und Paketstruktur', 1, 0, "Pakete liegen in src/ eines Workspaces; colcon erzeugt daneben build/, install/ und log/.", None),
    ("ament", "ament_cmake/ament_python", "werkzeuge", "04", 'Bauen mit colcon', 2, 0, "ament ist das Buildsystem für ein einzelnes Paket, mit CMake (C++) oder setuptools (Python).", None),
    ("colcon", "colcon build", "werkzeuge", "04", 'Bauen mit colcon', 1, 1, "colcon baut alle Pakete eines Workspaces in der richtigen Abhängigkeitsreihenfolge.", "„colcon ist der Compiler.“"),
    ("pkgcreate", "ros2 pkg create", "werkzeuge", "04", 'Schritt 1: Eigener Nachrichtentyp', 2, 1, "Erzeugt ein Paketgerüst mit Build-Type, Abhängigkeiten und optional einem Knoten.", None),
    ("custommsg", "Eigener Nachrichtentyp", "ros", "04", 'Schritt 1: Eigener Nachrichtentyp', 1, 0, "Eigene .msg-Dateien werden per rosidl_generate_interfaces in C++/Python-Code übersetzt — immer in einem CMake-Paket.", "„Ein Python-Projekt kann seine Nachrichtentypen im Python-Paket definieren.“"),
    ("setupbash", "source install/setup.bash", "werkzeuge", "04", 'Bauen mit colcon', 2, 1, "Erst das Sourcen des Install-Overlays macht neu gebaute Pakete im Terminal sichtbar.", "„Nach colcon build sind neue Pakete sofort in jedem Terminal verfügbar.“"),
    ("rosbag", "ros2 bag", "werkzeuge", "04", 'Ausblick: Daten aufzeichnen', 1, 1, "ros2 bag record/play zeichnet Topics auf und spielt sie ab — reproduzierbare Tests ohne reale Sensoren.", "„Am besten zeichnet man einfach immer alle Topics auf.“"),
    ("launch", "Launch-Datei", "ros", "04", 'Systeme starten mit Launch-Dateien', 1, 0, "Python-Launch-Files starten und parametrieren mehrere Knoten mit Argumenten, Remappings und Includes.", "„Ein Remapping in der Launch-Datei gilt automatisch für alle Knoten.“"),
    ("entrypoint", "entry_points (Python)", "werkzeuge", "04", 'Schritt 3: Subscriber in Python', 2, 1, "Python-Knoten werden in setup.py unter console_scripts als ausführbare Programme angemeldet.", "„Für Python-Knoten braucht man add_executable in der CMakeLists.txt.“"),
    ("logging", "Logging & Log-Level", "ros", "04", 'Logging', 2, 1, "Logger-Makros wie RCLCPP_INFO und einstellbare Log-Level strukturieren Debug-Ausgaben; --log-level knoten:=debug wirkt gezielt.", None),
    ("lifecycle", "Lifecycle-Knoten", "ros", "04", 'Knoten mit Lebenszyklus', 1, 1, "Managed Nodes durchlaufen den Zustandsautomaten Unconfigured, Inactive, Active und Finalized.", None),
    # --- VL 05 Bildverarbeitung
    ("image_msg", "sensor_msgs/Image", "ros", "05", 'Blick nach vorn - Bilder in ROS2', 2, 1, "Standardnachricht für unkomprimierte Kamerabilder mit Header, Größe, Encoding und flachem Byte-Array.", None),
    ("bildmatrix", "Bild als Pixelmatrix", "wahrnehmung", "05", 'Was ist ein Bild?', 1, 0, "Ein digitales Bild ist eine Matrix mit Ursprung oben links; der Zugriff erfolgt als image[Zeile, Spalte] = image[y, x].", "„image[x, y] liefert den Pixel an der Position (x, y).“"),
    ("hsv", "HSV-Farbraum", "wahrnehmung", "05", 'Farbräume', 1, 0, "HSV trennt Farbton von Helligkeit und macht farbbasierte Segmentierung — etwa von Ampelfarben — robuster.", "„Rot lässt sich mit einem einzigen Hue-Intervall erfassen.“"),
    ("thresholding", "Schwellwertoperation", "wahrnehmung", "05", 'Schwellwertoperationen (Thresholding)', 1, 0, "Binärisierung per festem oder adaptivem Schwellwert, z. B. weiße Fahrbahnlinien auf schwarzem Grund.", "„Ein fester Schwellwert funktioniert bei jeder Beleuchtung.“"),
    ("morphologie", "Morphologische Operationen", "wahrnehmung", "05", 'Morphologische Operationen', 2, 0, "Erosion, Dilatation, Opening und Closing bereinigen Binärmasken von Störpixeln und schließen Lücken.", None),
    ("faltung", "Faltung (Convolution)", "wahrnehmung", "05", 'Lineare Filter: Faltung (Convolution)', 2, 0, "Ein Kernel wird über das Bild geschoben und gewichtet die Nachbarschaft — Grundlage für Glättung, Kantenfilter und CNNs.", None),
    ("gaussfilter", "Gaußfilter", "wahrnehmung", "05", 'Gaußscher Filter', 2, 0, "Glättet Bildrauschen durch gaußförmig gewichtete Mittelung, verwischt dabei aber auch Kanten.", None),
    ("medianfilter", "Median-Filter", "wahrnehmung", "05", 'Median-Filter', 2, 0, "Ersetzt jeden Pixel durch den Median seiner Umgebung und entfernt Salt-and-Pepper-Rauschen kantenschonend.", "„Ein Gaußfilter entfernt Salt-and-Pepper-Rauschen genauso gut.“"),
    ("sobel", "Sobel-Operator", "wahrnehmung", "05", 'Sobel-Operator', 2, 0, "Approximiert die Bildgradienten in x- und y-Richtung; daraus ergeben sich Kantenstärke und -richtung.", None),
    ("canny", "Canny-Kantendetektor", "wahrnehmung", "05", 'Canny-Kantendetektor', 1, 0, "Gaußglättung, Sobel-Gradient, Non-Maximum-Suppression und Hysterese mit zwei Schwellwerten.", None),
    ("cv_bridge", "cv_bridge", "ros", "05", 'cv_bridge – Die Brücke zwischen ROS und OpenCV', 1, 1, "Konvertiert sensor_msgs/Image in OpenCV-Bilder und zurück (imgmsg_to_cv2, cv2_to_imgmsg).", None),
    ("compressed_image", "CompressedImage", "ros", "05", 'Komprimierte Bilder: sensor_msgs/CompressedImage', 2, 1, "JPEG-komprimierte Bildnachricht: weniger Last im WLAN, dafür CPU-Zeit und Qualitätsverlust.", None),
    # --- VL 06 Kamera
    ("pinhole", "Lochkameramodell", "mathe", "06", 'Abbildung eines 3D-Punkts auf die Bildebene', 1, 0, "Projiziert 3D-Punkte über ähnliche Dreiecke auf die Bildebene (u = f·X/Z) — die Tiefe geht verloren.", "„Aus einem einzelnen Kamerabild lässt sich die Entfernung direkt ablesen.“"),
    ("homprojektion", "Projektion in Matrixform", "mathe", "06", 'Homogene Koordinaten und Matrixform', 2, 0, "Homogene Koordinaten machen die Projektion zu einer Matrixmultiplikation mit anschließender Normierung.", None),
    ("kmatrix", "Kamera-Matrix K", "mathe", "06", 'Die Kamera-Matrix K', 1, 0, "Intrinsische 3×3-Matrix mit Brennweiten fx, fy und Hauptpunkt cx, cy.", "„K beschreibt auch die Linsenverzerrung.“"),
    ("verzerrung", "Verzerrungsparameter", "mathe", "06", 'Verzerrungsparameter', 2, 0, "Radiale und tangentiale Koeffizienten modellieren Tonnen- und Kissenverzeichnung realer Linsen.", None),
    ("entzerrung", "Entzerrung (undistort)", "wahrnehmung", "06", 'Anwendung der intrinsischen Parameter', 2, 1, "cv2.undistort rechnet mit K und D ein verzerrtes Kamerabild in ein ideales Lochkamerabild um.", None),
    ("kalibrierung", "Kamerakalibrierung", "wahrnehmung", "06", 'Intrinsische Kalibrierung mit Schachbrettmuster', 1, 0, "Aus vielen Schachbrettaufnahmen werden K und die Verzerrungskoeffizienten D bestimmt.", None),
    ("extrinsik", "Extrinsische Parameter", "mathe", "06", 'Extrinsische Parameter', 2, 0, "Rotation R und Translation t beschreiben die Lage der Kamera relativ zu Welt oder Roboter.", None),
    ("disparitaet", "Disparität & Tiefe", "wahrnehmung", "06", 'Prinzip der Stereo-Vision', 1, 0, "Zwei Kameras mit Basislinie b liefern über die Disparität d die Tiefe Z = f·b/d.", "„Große Disparität bedeutet ein weit entferntes Objekt.“"),
    ("rektifizierung", "Stereo-Rektifizierung", "wahrnehmung", "06", 'Vorverarbeitung', 2, 0, "Beide Bilder werden so gedreht, dass korrespondierende Punkte in derselben Zeile liegen.", None),
    ("camerainfo", "CameraInfo", "ros", "06", 'Kameraparameter in ROS2', 2, 1, "sensor_msgs/CameraInfo überträgt K, D, R und P einer kalibrierten Kamera.", None),
    ("pointcloud2", "PointCloud2", "ros", "06", 'Das PointCloud2-Message-Format', 1, 1, "ROS-2-Format für 3D-Punktwolken: ein Byte-Strom, dessen Aufbau PointField-Einträge beschreiben.", None),
    ("pc_transform", "Punktwolke transformieren", "ros", "06", '4. Koordinatensystem-Transformation', 2, 1, "Punktwolken liegen im Kamera-Frame vor und müssen per tf2 z. B. nach base_link überführt werden.", "„Die Punktwolke liegt automatisch im Roboterkoordinatensystem vor.“"),
    # --- VL 07 Objekterkennung
    ("detektion", "Detektion vs. Klassifikation", "wahrnehmung", "07", 'Begriffsdefinitionen', 1, 0, "Detektion: wo? (Bounding Box). Klassifikation: was? Tracking: wie bewegt es sich?", None),
    ("ecken", "Eckendetektoren", "wahrnehmung", "07", 'Corner Detection: Harris & Shi-Tomasi & FAST', 2, 0, "Harris und Shi-Tomasi finden Ecken über die Strukturmatrix, FAST über einen schnellen Pixelkreistest.", None),
    ("orb", "ORB-Deskriptor", "wahrnehmung", "07", 'Feature Descriptoren: Rotated BRIEF', 2, 0, "FAST-Keypoints mit orientierungskorrigiertem binärem Deskriptor — rotationsinvariant.", None),
    ("matching", "Feature Matching", "wahrnehmung", "07", 'Feature Matching', 2, 1, "Deskriptoren werden per Distanz zugeordnet (cv2.BFMatcher), Ausreißer per RANSAC entfernt.", "„Ähnliche Deskriptoren bedeuten dasselbe Objekt.“"),
    ("cnn", "CNN", "wahrnehmung", "07", 'CNN-Grundlagen (Kurzer Einstieg)', 1, 0, "Faltungs-, Pooling- und Fully-Connected-Schichten lernen hierarchische Bildmerkmale aus Trainingsdaten.", "„Die Filterkerne eines CNN werden von Hand vorgegeben.“"),
    ("yolo", "YOLO", "wahrnehmung", "07", 'YOLO: You Only Look Once', 1, 0, "Single-Shot-Detektor: Bounding Boxes, Konfidenzen und Klassen in einem Durchlauf; NMS entfernt Doppelte.", None),
    ("ultralytics", "YOLO mit ultralytics", "wahrnehmung", "07", 'YOLOv8 in der Anwendung', 2, 1, "Die ultralytics-API lädt ein vortrainiertes Modell und liefert Boxen mit Klassen.", "„Ein auf COCO vortrainiertes Modell erkennt jedes Objekt ohne Nachtraining — auch Verkehrsschilder der Verkehrswelt.“"),
    ("bbox3d", "Bounding Box → 3D-Position", "wahrnehmung", "07", 'Von Bounding Box zu 3D-Position', 1, 0, "Box-Mittelpunkt, Tiefe und K ergeben die Position des Objekts in Kamerakoordinaten.", None),
    ("optflow", "Optischer Fluss", "wahrnehmung", "07", 'Optical Flow: Lucas-Kanade', 2, 0, "Lucas-Kanade schätzt kleine Pixelverschiebungen aus räumlichen und zeitlichen Gradienten.", None),
    ("deepsort", "DeepSORT-Tracking", "schaetzung", "07", 'DeepSORT: Deep Simple Online Realtime Tracking', 2, 0, "Multi-Object-Tracking mit Kalman-Prädiktion, Appearance-Features und ungarischer Zuordnung.", None),
    ("ransac", "RANSAC", "wahrnehmung", "07", 'RANSAC: Plane Segmentation', 1, 0, "Robuste Modellschätzung aus zufälligen Minimalstichproben — das Modell mit den meisten Inliern gewinnt.", None),
    ("icp", "ICP-Registrierung", "wahrnehmung", "07", 'ICP: Iterative Closest Point', 2, 0, "Richtet zwei Punktwolken iterativ über nächste Nachbarn und eine Starrkörpertransformation aus.", "„ICP findet von jeder Startlage aus die richtige Ausrichtung.“"),
    # --- VL 08 Sensorfusion I
    ("sensorfusion", "Sensordatenfusion", "schaetzung", "08", 'Grundlagen und Konzepte', 1, 0, "Kombination mehrerer Quellen zu einer Schätzung, die genauer oder vollständiger ist als jede einzelne.", None),
    ("fusionsarten", "Fusionsarten", "schaetzung", "08", 'Grundlagen und Konzepte', 2, 0, "Konkurrierend (genauer), komplementär (vollständiger) oder kooperativ (neue Information).", None),
    ("fusionsebenen", "Fusionsebenen", "schaetzung", "08", 'Grundlagen und Konzepte', 2, 0, "Fusion auf Rohdaten-, Merkmals- oder Entscheidungsebene; je später, desto stärker vorverarbeitet.", None),
    ("bayesfilter", "Diskreter Bayes-Filter", "schaetzung", "08", 'Einführung in den diskreten Bayes Filter', 1, 0, "Der Zustand als Wahrscheinlichkeitsverteilung über Zellen, zyklisch aktualisiert durch Bewegung und Messung.", None),
    ("sensormodell", "Sensormodell", "wahrnehmung", "08", 'Bildsensor', 2, 0, "Wie wahrscheinlich ist eine Messung in jedem Zustand — einschließlich Falsch-Positiv und Falsch-Negativ?", None),
    ("apriori", "Prior, Likelihood, Posterior", "mathe", "08", 'Abbildung auf apriori Wissen', 1, 0, "Posterior = Likelihood × Prior, normiert: Eine Messung gewichtet das bisherige Wissen neu.", "„Eine neue Messung ersetzt einfach die bisherige Schätzung.“"),
    ("bewegfaltung", "Bewegung als Faltung", "schaetzung", "08", 'Bewegung', 2, 0, "Der Predict-Schritt verschiebt die Verteilung und verschmiert sie — die Unsicherheit wächst.", "„Nach einer Bewegung weiß der Roboter genauer, wo er ist.“"),
    ("predictupdate", "Predict-Update-Zyklus", "schaetzung", "08", 'Und jetzt alles zusammen', 1, 0, "Vorhersage (Unsicherheit wächst) und Korrektur durch Messung (Unsicherheit sinkt) im Wechsel.", None),
    ("gridlokalisierung", "Grid-Lokalisierung", "schaetzung", "08", 'Erweiterung auf 2D: Roboter-Lokalisierung im Grid', 2, 0, "Aus Karte, Landmarken und Bewegungsmodell konvergiert eine 2D-Verteilung zur Pose.", "„Eine Landmarkenerkennung reicht für eine eindeutige Lokalisierung.“"),
    ("mapmmse", "MAP vs. Erwartungswert", "mathe", "08", 'Ausblick: Vom diskreten zum kontinuierlichen Filter', 2, 0, "argmax liefert springende Schätzungen; der Erwartungswert nutzt die ganze Verteilung.", None),
    # --- VL 09 Sensorfusion II
    ("qos", "Quality of Service", "ros", "09", 'Einschub - Quality of Service (QoS) für Sensordaten', 2, 1, "QoS-Profile (Reliability, Durability, History, Depth) steuern die Zustellung; Sensordaten meist best effort.", "„Ein Subscriber empfängt jedes Topic, egal mit welchen QoS-Einstellungen.“"),
    ("normalverteilung", "Normalverteilung", "mathe", "09", 'Warum Normalverteilungen?', 1, 0, "Gaußverteilungen bleiben bei Predict und Update gaußförmig — μ und σ² genügen.", None),
    ("kalman", "Kalman-Filter", "schaetzung", "09", 'Der eindimensionale Kalman-Filter', 1, 0, "Optimaler Schätzer für lineare Systeme mit Gaußrauschen — der Predict-Update-Zyklus in geschlossener Form.", None),
    ("kalmangain", "Kalman-Gain", "schaetzung", "09", 'Der Kalman-Gain im Detail', 1, 0, "Gewichtet die Innovation nach dem Verhältnis von Vorhersage- zu Messunsicherheit.", "„Der Kalman-Gain ist ein fester Parameter.“"),
    ("qr", "Prozess- vs. Messrauschen", "schaetzung", "09", 'Variation der Parameter', 2, 0, "Q und R legen fest, wie sehr der Filter dem Modell bzw. dem Sensor vertraut.", "„Kleinere Rauschwerte machen den Filter immer genauer.“"),
    ("kfmatrix", "Mehrdimensionaler KF", "schaetzung", "09", 'Der mehrdimensionale Kalman-Filter', 2, 0, "Mit Zustandsvektor und Kovarianzmatrix lassen sich auch nicht gemessene Größen schätzen.", "„Der Filter schätzt nur, was ein Sensor direkt misst.“"),
    ("ekf", "Extended Kalman Filter", "schaetzung", "09", 'Extended Kalman Filter (EKF)', 1, 0, "Linearisiert nichtlineare Bewegungs- und Messmodelle um die aktuelle Schätzung.", "„Der EKF ist auch für nichtlineare Systeme exakt optimal.“"),
    ("jacobi", "Jacobi-Matrix", "mathe", "09", 'Linearisierung durch Taylor-Entwicklung', 2, 0, "Taylor-Entwicklung erster Ordnung; die Jacobi-Matrizen propagieren im EKF die Kovarianz.", None),
    ("robotlocalization", "robot_localization", "ros", "09", 'robot_localization in ROS 2', 1, 1, "ROS-2-Paket, das Odometrie, IMU und GPS per EKF zu /odometry/filtered fusioniert.", None),
    ("ekfconfig", "EKF-Konfiguration", "ros", "09", 'Konfiguration des EKF-Nodes', 2, 1, "In ekf.yaml legt pro Sensor eine Bool-Liste fest, zu welchen Zustandsgrößen er beiträgt.", None),
    ("kovarianzpraxis", "Sensor-Kovarianzen", "schaetzung", "09", 'Tipps für die Praxis', 2, 0, "Realistische Kovarianzen steuern das Sensorvertrauen; falsche Frames oder Zeitstempel sabotieren die Fusion.", "„Kovarianz 0 bedeutet einen perfekten Sensor.“"),
    # --- VL 10 Kinematik und Regelung
    ("diffdrive", "Differentialantrieb", "mathe", "10", 'Differentialantrieb (Differential Drive)', 1, 0, "Zwei unabhängig angetriebene Räder; die Kinematik verbindet Radgeschwindigkeiten mit v und ω.", None),
    ("nichtholonom", "Nichtholonome Einschränkung", "mathe", "10", 'Differentialantrieb (Differential Drive)', 2, 0, "Ein Differentialantrieb kann nicht seitwärts fahren — wichtig beim Einparken.", "„Wer auf der Stelle drehen kann, kann jede Pose direkt anfahren.“"),
    ("radodometrie", "Rad-Odometrie", "schaetzung", "10", 'Bewegung direkt messen - Odometrie', 1, 0, "Aus Encoder-Ticks und Kinematik aufintegrierte Pose; die Fehler summieren sich zur Drift.", "„Rad-Odometrie liefert dauerhaft eine zuverlässige Position.“"),
    ("regelung", "Steuerung vs. Regelung", "bewegung", "10", 'Steuerung vs. Regelung', 1, 0, "Eine Regelung misst die Regelgröße laufend zurück und vergleicht sie mit dem Sollwert.", "„Fahren mit fester Geschwindigkeit und Dauer ist schon eine Regelung.“"),
    ("strecke", "Streckenverhalten", "bewegung", "10", 'Streckenverhalten', 2, 0, "Regelstrecken werden als P-, I-, Totzeit-, PT1- oder PT2-Glied modelliert.", None),
    ("pid", "PID-Regler", "bewegung", "10", 'Der PID-Regler', 1, 0, "Proportionaler, integrierender und differenzierender Anteil auf die Regelabweichung e = w − r.", "„Ein höherer P-Anteil macht die Regelung immer besser.“"),
    ("guete", "Gütekriterien", "bewegung", "10", 'Reglerperformanz und Gütekriterien', 2, 0, "Stabilität, Störkompensation, Sollwertfolge und Robustheit bewerten einen Regelkreis.", None),
    ("movetopose", "Move-to-Pose", "bewegung", "10", 'Move-to-Pose Controller', 2, 0, "Regelt Abstand und Winkel zur Zielpose — für Punkt-zu-Punkt-Fahrten ohne Hindernisse.", None),
    ("purepursuit", "Pure Pursuit", "bewegung", "10", 'Pure Pursuit Controller', 1, 0, "Folgt einem Pfad über einen Kreisbogen zu einem Lookahead-Punkt auf der Bahn.", "„Je kleiner die Lookahead-Distanz, desto besser.“"),
    ("twist", "Twist / cmd_vel", "ros", "10", 'Twist-Messages', 1, 1, "geometry_msgs/Twist auf cmd_vel ist die Schnittstelle für Fahrbefehle; beim TurtleBot zählen linear.x und angular.z.", None),
    ("ros2control", "ros2_control", "ros", "10", 'ros2_control', 2, 1, "Framework zwischen Controllern und Hardware; der diff_drive_controller rechnet cmd_vel in Radgeschwindigkeiten um.", None),
    # --- VL 11 Navigation
    ("navigation", "Navigation", "bewegung", "11", 'Definitionen', 1, 0, "Selbstlokalisierung, bei Bedarf Kartenerstellung und Wegplanung zum Ziel.", None),
    ("bug", "Bug-Algorithmen", "bewegung", "11", 'Bug-Algorithmen', 2, 0, "Reaktive Strategien ohne Karte aus Zielrichtung und Konturfolge.", "„Direkt aufs Ziel zufahren und Hindernisse umfahren führt sicher ans Ziel.“"),
    ("occupancygrid", "Occupancy Grid", "bewegung", "11", 'Rasterbasierte Karten (Occupancy Grids)', 1, 0, "Rasterkarte mit Belegungswahrscheinlichkeit je Zelle — in ROS nav_msgs/OccupancyGrid.", None),
    ("quadtree", "Quadtree", "bewegung", "11", 'Adaptive Auflösung: Quadtrees', 2, 0, "Teilt inhomogene Zellen rekursiv und spart Speicher in offenen Bereichen.", None),
    ("costmap", "Costmap", "ros", "11", 'Costmaps -- Planungskarten in Nav2', 1, 0, "Nav2-Planungskarte mit Kosten von 0 bis 255 aus gestapelten Layern, global und lokal.", "„Die Costmap ist einfach die gespeicherte Karte.“"),
    ("inflation", "Inflation Layer", "ros", "11", 'Costmaps -- Planungskarten in Nav2', 2, 0, "Bläst Hindernisse mit abfallenden Kosten auf — für Sicherheitsabstand zum Roboter-Footprint.", None),
    ("topokarte", "Topologische Karte", "bewegung", "11", 'Topologische Karten', 2, 0, "Graph aus Orten und Verbindungen: kompakt, aber ohne metrische Genauigkeit.", None),
    ("bfs", "Breitensuche", "bewegung", "11", 'Breitensuche (Wavefront)', 2, 0, "Breitet sich wellenförmig vom Start aus; vollständig, aber ohne Richtung zum Ziel.", None),
    ("dijkstra", "Dijkstra", "bewegung", "11", 'Dijkstra-Algorithmus', 2, 0, "Erweitert Knoten nach aufsummierten Kosten; optimal bei nicht-negativen Kantengewichten.", None),
    ("astar", "A*", "bewegung", "11", 'A*-Algorithmus', 1, 0, "Sortiert nach f = g + h; mit zulässiger Heuristik optimal und meist schneller als Dijkstra.", "„Eine großzügige Heuristik macht A* nur schneller, nicht schlechter.“"),
    ("nav2", "Nav2", "ros", "11", 'ROS2 Navigation Stack (Nav2)', 1, 0, "Navigationsframework aus Planner-, Controller- und Behavior-Servern, koordiniert durch Behavior Trees.", None),
]

# ---- Beziehungen: von, nach, Beschriftung („von … text … nach“)
R = [
    ("roboter", "klassifikation", "wird geordnet durch"),
    ("pkgcreate", "entrypoint", "Python-Gerüst mit"),
    ("topic", "kommmuster", "ergänzt durch"),
    ("topiccli", "tbtopics", "untersucht"),
    ("tbtopics", "twist", "Fahrbefehl als"),
    ("sta", "turtlebot", "konkret am"),
    ("herausforderungen", "rosmotivation", "begründet"),
    ("verkehrswelt", "turtlebot", "gelöst mit"),
    ("shell", "pfade", "navigiert über"),
    ("shell", "pipes", "verknüpft mit"),
    ("rechte", "path", "Voraussetzung für"),
    ("umgebung", "path", "Beispiel"),
    ("umgebung", "source", "bleibt erhalten mit"),
    ("source", "bashrc", "dauerhaft in"),
    ("ssh", "turtlebot", "Zugang zum"),
    ("prozess", "umgebung", "erbt"),
    ("imu", "drift", "leidet unter"),
    ("odometrie", "pose", "schätzt"),
    ("homtrafo", "verkettung", "ermöglicht"),
    ("quaternion", "euler", "vermeidet Schwächen von"),
    ("verkettung", "tftree", "realisiert im"),
    ("frameid", "tf2", "genutzt von"),
    ("dds", "discovery", "realisiert"),
    ("node", "topic", "publiziert auf"),
    ("topic", "message", "überträgt"),
    ("msgdef", "custommsg", "Grundlage für"),
    ("colcon", "ament", "orchestriert"),
    ("rosbag", "topic", "zeichnet auf"),
    ("hsv", "thresholding", "liefert Eingabe für"),
    ("thresholding", "morphologie", "bereinigt durch"),
    ("gaussfilter", "canny", "erster Schritt von"),
    ("faltung", "cnn", "Grundlage für"),
    ("image_msg", "cv_bridge", "konvertiert durch"),
    ("kalibrierung", "kmatrix", "bestimmt"),
    ("kmatrix", "camerainfo", "übertragen in"),
    ("disparitaet", "pointcloud2", "liefert Tiefe für"),
    ("yolo", "bbox3d", "liefert Box für"),
    ("disparitaet", "bbox3d", "liefert Tiefe für"),
    ("yolo", "deepsort", "liefert Detektionen für"),
    ("pointcloud2", "pc_transform", "erfordert"),
    ("bayesfilter", "predictupdate", "folgt dem"),
    ("normalverteilung", "kalman", "ermöglicht"),
    ("bayesfilter", "kalman", "Spezialfall"),
    ("ekf", "jacobi", "linearisiert mit"),
    ("ekf", "robotlocalization", "implementiert in"),
    ("radodometrie", "robotlocalization", "Eingang für"),
    ("diffdrive", "radodometrie", "Kinematik für"),
    ("regelung", "pid", "realisiert durch"),
    ("purepursuit", "twist", "erzeugt"),
    ("dijkstra", "astar", "+ Heuristik"),
    ("costmap", "astar", "Grundlage für"),
    ("purepursuit", "nav2", "Controller in"),
    # --- über Kapitel hinweg
    ("rosmotivation", "middleware", "konkret als"),
    ("sta", "sensor", "Sense-Ebene"),
    ("shell", "ros2cli", "bedient"),
    ("source", "setupbash", "angewandt auf"),
    ("umgebung", "domainid", "Beispiel"),
    ("pfade", "workspace", "Struktur des"),
    ("turtlebot", "domainid", "eine pro Gruppe"),
    ("topic", "rosbag", "aufgezeichnet von"),
    ("message", "image_msg", "Beispiel"),
    ("discovery", "qos", "nur bei passender"),
    ("frameid", "pc_transform", "nötig für"),
    ("homtrafo", "homprojektion", "angewandt in"),
    ("extrinsik", "tf2", "veröffentlicht über"),
    ("faltung", "bewegfaltung", "gleiche Operation"),
    ("odometrie", "radodometrie", "berechnet als"),
    ("drift", "sensorfusion", "motiviert"),
    ("pose", "ekf", "geschätzt von"),
    ("imu", "robotlocalization", "Eingang für"),
    ("tf2", "robotlocalization", "Frames für"),
    ("ransac", "matching", "säubert"),
    ("occupancygrid", "costmap", "Grundlage für"),
    ("twist", "turtlebot", "steuert"),
]

# ---- Didaktische Fäden (Metrolinien): jede Folge ist eine Kette von „baut auf“-Schritten
F = [
    ("f_projekt", "Projektaufgabe", "Vom Ziel in der ersten Vorlesung zum autonom fahrenden TurtleBot in der Verkehrswelt.",
     ["verkehrswelt", "turtlebot", "domainid", "rosbag", "thresholding", "kalibrierung", "yolo", "robotlocalization", "purepursuit", "costmap", "nav2"]),
    ("f_umgebung", "Umgebung im Griff", "Von der Shell über Umgebungsvariablen zum lauffähigen ROS-2-Workspace.",
     ["shell", "umgebung", "source", "bashrc", "ros2cli", "setupbash", "domainid"]),
    ("f_ros", "Vom Knoten zum System", "Wie aus einzelnen Knoten und Nachrichten ein vollständiges Robotersystem wird.",
     ["rosmotivation", "middleware", "node", "topic", "message", "paket", "launch", "tf2", "nav2"]),
    ("f_koord", "Koordinaten überall", "Bezugssysteme und Transformationen ziehen sich vom Roboter bis zur Kamera und zum Fahrwerk.",
     ["frame", "homtrafo", "verkettung", "tf2", "pose", "kmatrix", "pc_transform", "bbox3d", "diffdrive"]),
    ("f_bild", "Vom Pixel zum Objekt", "Von der Pixelmatrix bis zum erkannten Verkehrszeichen mit 3D-Position.",
     ["bildmatrix", "hsv", "thresholding", "canny", "kalibrierung", "disparitaet", "cnn", "yolo", "bbox3d"]),
    ("f_unsicher", "Unsicherheit beherrschen", "Kein Sensor ist perfekt: von Sensorfehlern über Bayes zur fusionierten Pose.",
     ["sensorparam", "drift", "sensorfusion", "bayesfilter", "predictupdate", "kalman", "ekf", "robotlocalization"]),
]

# ---- Prüfen
ids = [b[0] for b in B]
assert len(set(ids)) == len(ids), ("doppelte id", [i for i in ids if ids.count(i) > 1])
term_of = {b[0]: IDX[b[3]] for b in B}
for fid, _, _, st in F:
    for x in st:
        assert x in term_of, (fid, x)
    for u, v in zip(st, st[1:]):
        assert term_of[u] <= term_of[v], ("Faden nicht chronologisch", fid, u, v)
for a, b, _ in R:
    assert a in term_of and b in term_of, (a, b)
linien = {l[0] for l in LINIEN}
for b in B:
    assert b[2] in linien, ("unbekannte Linie", b)

def abschnitt_nr(key, titel):
    heads = sections[key]
    if titel not in heads:
        raise SystemExit(f"Abschnitt nicht gefunden: {key} »{titel}« — vorhanden: {heads}")
    return heads.index(titel) + 1

phasen, start = [], 0
for i in range(1, len(T) + 1):
    if i == len(T) or T[i][6] != T[start][6]:
        blk = T[start][6]
        phasen.append({"von": start, "bis": i - 1, "name": BLOECKE[blk], "farbe": BLOCKFARBE[blk]})
        start = i

out = {
    "stand": STAND,
    "kurs": KURS,
    "linien": [{"id": i, "name": n} for i, n in LINIEN],
    "phasen": phasen,
    "vorlesungen": [
        {"vl": i, "key": t[0], "label": t[1], "datei": t[4], "datum": t[2], "kurz": t[3],
         "status": t[5], "block": t[6], "abschnitte": sections[t[0]]}
        for i, t in enumerate(T)
    ],
    "begriffe": [
        {"id": b[0], "name": b[1], "linie": b[2], "vl": IDX[b[3]], "abschnitt": abschnitt_nr(b[3], b[4]),
         "ebene": b[5], "code": bool(b[6]), "erklaerung": b[7], "fehlvorstellung": b[8]}
        for b in B
    ],
    "beziehungen": [{"von": a, "nach": b, "text": t} for a, b, t in R],
    "faeden": [{"id": f, "name": n, "beschreibung": d, "stationen": st} for f, n, d, st in F],
}
os.makedirs(ZIEL, exist_ok=True)
text = json.dumps(out, ensure_ascii=False, indent=1)
open(os.path.join(ZIEL, "kurslandkarte.json"), "w", encoding="utf-8").write(text + "\n")
seite = open(os.path.join(HIER, "vorlage.html"), encoding="utf-8").read()
seite = seite.replace("%%DATEN%%", text.replace("</", "<\\/")).replace("%%STAND%%", STAND)
open(os.path.join(ZIEL, "index.html"), "w", encoding="utf-8").write(seite)
e1 = sum(1 for b in B if b[5] == 1)
print(f"{len(T)} Termine, {len(B)} Begriffe ({e1} Kern, {len(B)-e1} Detail), {len(R)} Beziehungen, {len(F)} Fäden")
