<!--

author:   Sebastian Zug & Claude Code (Opus 5.5) mit LiaScript Teaching-Agent
email:    sebastian.zug@informatik.tu-freiberg.de
version:  0.1.0
language: de
narrator: Deutsch Female

import:   https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/config.md

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/exercises/praktikum_01_turtlebot/README.md)

# Praktikum 1: TurtleBot in Betrieb nehmen

<!-- data-type="none" -->
| Parameter            | Kursinformationen                                                                                                       |
| -------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| **Veranstaltung:**   | @config.lecture                                                                                                         |
| **Semester**         | @config.semester                                                                                                        |
| **Hochschule:**      | `Technische Universität Freiberg`                                                                                       |
| **Inhalte:**         | `Verbinden, untersuchen, fahren und aufzeichnen am TurtleBot 3 — Anwendung von Block 1`                                 |
| **Link auf GitHub:** | https://github.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/blob/master/exercises/praktikum_01_turtlebot/README.md  |
| **Autoren**          | @author                                                                                                                 |

--------------------------------------------------------------------------------

**Heute zum ersten Mal am echten Roboter.** Sie lernen nichts Neues — Sie wenden an, was Sie in Block 1 gelernt haben. Am Ende haben Sie Ihren TurtleBot verbunden, untersucht, gesteuert und eine Fahrt durch die Verkehrswelt aufgezeichnet. Diese Aufzeichnung ist das Ausgangsmaterial für Ihre Spurerkennung in Block 2.

**Ziele** — am Ende dieses Praktikums können Sie als Gruppe

+ Ihren Laptop mit dem TurtleBot verbinden und die Verbindung prüfen,
+ Topics, Nachrichtentypen, QoS und Transformationen des Roboters selbst ermitteln,
+ den Roboter sicher steuern und seine Sensordaten in RViz darstellen,
+ eine Fahrt als Bag aufzeichnen und ohne Roboter wieder abspielen.

**Ablauf (90 min)**

<!-- data-type="none" -->
| Zeit   | Teil                                     | Wiederholt aus           |
| ------ | ---------------------------------------- | ------------------------ |
| 5 min  | Sicherheit, Rollen                       | —                        |
| 15 min | A: Verbinden                             | Ü 1, VL 2, VL 3          |
| 20 min | B: Den Roboter untersuchen + **Zwischenabnahme** | VL 2, VL 4       |
| 15 min | C: Fahren und Sensordaten ansehen        | VL 2, VL 4               |
| 20 min | D: Eine Fahrt aufzeichnen                | Ü 2                      |
| 10 min | Abnahme, Roboter zurückgeben             | —                        |
| 5 min  | Puffer                                   |                          |

**Vor dem Termin zu Hause erledigen** (spart im Labor die meiste Zeit): Paket aus der Vorbereitung installieren, bei WSL2 den Netzwerkmodus umstellen und den Selbsttest machen.

**Wo P 1 im Projekt steht:** P 1 Daten aufnehmen → P 2 Kamerabild korrigieren → P 3 Fahrspur erkennen → P 4 Hindernisse erkennen → … → Block 6: Ihr TurtleBot fährt autonom durch die Verkehrswelt.

--------------------------------------------------------------------------------

## Vorbereitung und Sicherheit

    --{{0}}--
Bevor der Roboter fährt, klären wir drei Dinge: wer in der Gruppe welche Aufgabe übernimmt, wie Sie sicher mit dem Roboter umgehen, und wie Ihr Laptop ins Netz des Roboters kommt.

**Mitbringen:** Laptop mit ROS 2 Jazzy (Selbsttest der [Installationsanleitung](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/exercises/installation_ros2/README.md) bestanden). Installieren Sie zusätzlich:

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
```

`teleop_twist_keyboard` funktioniert wie `turtle_teleop_key` aus VL 2 — nur für echte Roboter.

**Sicherheit**

+ Der Roboter fährt auf dem Tisch bzw. im Parcours — **eine Person hat immer die Hand in der Nähe** und hebt ihn bei Gefahr hoch.
+ Höchstgeschwindigkeit heute: **0,15 m/s**.
+ Teleop sendet beim Beenden mit `Ctrl+C` einen Stoppbefehl. Hängt das Programm oder bricht das WLAN ab, fährt der Roboter mit dem letzten Befehl weiter — halten Sie deshalb einen eigenen Stoppbefehl bereit (Aufgabe C.1) und im Zweifel den Roboter hoch.
+ Akku: Ist die Spannung zu niedrig, verhält sich der Roboter unberechenbar. Melden Sie das den Betreuern.

**Rollen** — je Teil wechseln, damit alle alles einmal machen:

<!-- data-type="none" -->
| Rolle          | Aufgabe                                                     |
| :------------- | :---------------------------------------------------------- |
| Tastatur       | tippt die Befehle, in C: steuert den Roboter (Sicherheitsverantwortung) |
| Nachschlagen   | sucht in den Unterlagen von Block 1 den passenden Befehl, prüft die Ausgabe |
| Protokoll      | hält die 📝-Antworten fest                                  |

In Zweiergruppen übernimmt die Person am Nachschlagen auch das Protokoll.

**Protokoll:** Führen Sie die Datei `praktikum_01/PROTOKOLL.md` lokal und tragen Sie alle Antworten ein, die mit 📝 markiert sind. Im WLAN des Roboters haben Sie in der Regel kein Internet — laden Sie das Protokoll deshalb **nach** dem Praktikum ins Git-Repository Ihrer Gruppe hoch, spätestens zu Beginn von P 2.

**Nur für WSL2 — Pflicht vor dem Praktikum:** Damit Ihr Laptop den Roboter im WLAN sieht, stellen Sie WSL auf den gespiegelten Netzwerkmodus um (einmalig, zu Hause):

1. Unter Windows die Datei `%UserProfile%\.wslconfig` anlegen bzw. öffnen, z.B. in der PowerShell mit `notepad $env:USERPROFILE\.wslconfig`.
2. Eintragen und speichern:

   ```text
   [wsl2]
   networkingMode=mirrored
   ```

3. In der PowerShell `wsl --shutdown`, danach Ubuntu neu öffnen.
4. Selbsttest zu Hause: `ip addr` in Ubuntu zeigt jetzt dieselbe IP-Adresse wie `ipconfig` unter Windows.
5. Ist der Roboter im Praktikum trotzdem nicht sichtbar, blockiert meist die Windows-Firewall. Die Betreuer helfen — arbeiten Sie bis dahin am Laptop eines Gruppenmitglieds weiter.

## A: Verbinden

    --{{0}}--
Der Roboter läuft bereits: Auf seinem Raspberry Pi startet beim Einschalten ein ROS-2-Jazzy-System mit den Treibern für LiDAR, Kamera und Motoren. Sie müssen Ihren Laptop nur ins richtige Netz und in die richtige Domäne bringen.

**Ein funktionierender Laptop pro Gruppe genügt für den Start.** Die anderen verbinden sich parallel.

Auf dem TurtleBot läuft bereits ROS 2 Jazzy. Nach dem Einschalten starten die Treiber für LiDAR, Kamera und Motoren von selbst — so, wie es `robot.launch.py` aus VL 3 beschreibt.

**A.1 Netz und Domäne**

1. Verbinden Sie Ihren Laptop mit dem WLAN Ihres TurtleBots (Zugangsdaten von den Betreuern).
2. Die `ROS_DOMAIN_ID` Ihrer Gruppe steht auf dem Roboter. Ersetzen Sie in `~/.bashrc` den Übungswert `7` aus Ü 1 (Aufgabe 5) durch diese ID, z.B. mit `nano ~/.bashrc`. Kein Eintrag vorhanden? Dann ergänzen Sie ihn:

   ```bash
   echo "export ROS_DOMAIN_ID=<Ihre ID>" >> ~/.bashrc
   ```

3. Haben Sie für zu Hause `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` gesetzt (Installationsanleitung), kommentieren Sie diese Zeile in `~/.bashrc` für das Praktikum aus — sonst sucht Ihr Laptop nur bei sich selbst. Öffnen Sie danach ein **neues** Terminal.

**A.2 Verbindung prüfen**

```text
> ros2 topic list
/camera/image_raw
/cmd_vel
/odom
/scan
...
```

📝 Welche Topics stellt Ihr Roboter bereit? Notieren Sie die Liste.

<details>
<summary>Sie sehen nur `/parameter_events` und `/rosout`?</summary>

Dann findet Ihr Laptop den Roboter nicht. Prüfen Sie der Reihe nach:

1. `printenv | grep ROS` — stimmt die `ROS_DOMAIN_ID`, ist `LOCALHOST` noch gesetzt? (Ü 1, VL 3)
2. Sind Sie im richtigen WLAN?
3. WSL2: gespiegelter Netzwerkmodus aktiv (siehe Vorbereitung)?
4. Hilft das alles nicht: Betreuer fragen und am Laptop eines Gruppenmitglieds weiterarbeiten.

</details>

**A.3 Kommen die Daten an?**

```bash
ros2 topic hz /scan
ros2 topic hz /camera/image_raw
```

📝 Mit welcher Rate senden LiDAR und Kamera?

## B: Den Roboter untersuchen

    --{{0}}--
Bevor Sie Code für einen Roboter schreiben, müssen Sie seine Schnittstellen kennen. In diesem Teil ermitteln Sie selbst, was in keiner Anleitung zuverlässig steht: welche Typen, welche QoS und welche Koordinatensysteme Ihr Roboter verwendet.

Ein Datenblatt kann veralten — Ihr Roboter selbst nicht. Ermitteln Sie mit den Werkzeugen aus Block 1, wie er tatsächlich konfiguriert ist.

**B.1 Nachrichtentypen** (VL 2)

📝 Welchen Typ haben `/scan`, `/odom`, `/camera/image_raw` und `/cmd_vel`?

```bash
ros2 topic info /cmd_vel
```

Achten Sie besonders auf `/cmd_vel`: `geometry_msgs/msg/Twist` oder `geometry_msgs/msg/TwistStamped`? Davon hängt ab, wie Sie in C fahren — und wie Ihr Fahrverhalten später senden muss.

**B.2 Quality of Service** (VL 4)

```bash
ros2 topic info -v /scan
```

📝 Welche `Reliability` und welche `Durability` verwendet der LiDAR-Treiber? Was folgt daraus für RViz und für Ihre eigenen Knoten?

**B.3 Koordinatensysteme** (VL 4)

```bash
ros2 topic echo --once /scan | grep frame_id      # Pipe und grep aus Ü 1
ros2 run tf2_tools view_frames
```

📝 In welchem Frame (`frame_id`) liefert der LiDAR seine Messungen? Kommt dieser Frame im tf-Baum aus `view_frames` vor? Fügen Sie die PDF-Datei ins Protokoll ein.

📝 Fragen Sie mit `ros2 run tf2_ros tf2_echo base_footprint <Frame des LiDAR>` ab, wo der LiDAR sitzt. Stimmt der Wert mit dem Modell aus VL 4 überein (z = 0,182 m für `base_scan`)?

<details>
<summary>Der Frame des LiDAR fehlt im tf-Baum?</summary>

Das ist Ursache 1 aus der Leitfrage von VL 4. Meldet der LiDAR-Treiber einen Frame, den das Modell des Roboters nicht kennt (z.B. `laser`), ergänzen Sie die fehlende Verbindung selbst mit einer statischen Transformation. Sitzt der Sensor genau im Ursprung von `base_scan`, ist das eine Transformation ohne Verschiebung:

```bash
ros2 run tf2_ros static_transform_publisher --frame-id base_scan --child-frame-id <Frame des LiDAR>
```

Prüfen Sie danach erneut mit `tf2_echo`.

</details>

> **Zwischenabnahme** (noch mit Verbindung zum Roboter): Zeigen Sie den Betreuern `ros2 topic list` und `tf2_echo` mit der Lage des LiDAR (Abnahme Nr. 1 und 3).

## C: Fahren und Sensordaten ansehen

    --{{0}}--
Jetzt darf der Roboter fahren. Halten Sie sich an die Höchstgeschwindigkeit, und testen Sie zuerst den Stoppbefehl, bevor Sie mit der Tastatur fahren.

**C.1 Stoppbefehl vorbereiten** — **bevor** Sie fahren. Verwenden Sie den Typ aus B.1 (VL 2):

```bash
# für Twist:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
# für TwistStamped:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{}"
```

Alle Felder sind 0 — der Roboter steht. Lassen Sie den passenden Befehl in einem Terminal bereitstehen.

**C.2 Per Tastatur fahren**

Teleop startet ohne weitere Angaben mit **0,5 m/s** — viel zu schnell. Setzen Sie die Startgeschwindigkeit deshalb beim Start als Parameter (Ü 2):

```bash
# für Twist:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.5
# für TwistStamped:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.5 -p stamped:=true
```

Erhöhen Sie die Geschwindigkeit mit den Tasten des Programms nicht über 0,15 m/s.

📝 Welcher Wert für `linear.x` ergibt sich, wenn Sie geradeaus fahren? Prüfen Sie es mit `ros2 topic echo /cmd_vel`. (VL 2: `linear.x` ist vorwärts, `angular.z` die Drehung.)

**C.3 Sensordaten ansehen**

*Kamerabild* mit `rqt` aus VL 2: Menü _Plugins → Visualization → Image View_, oben das Topic `/camera/image_raw` wählen.

```bash
rqt
```

*LiDAR in RViz* — mit den drei Schritten aus VL 4:

```bash
rviz2
```

Erscheinen keine Punkte, prüfen Sie die beiden Ursachen aus VL 4: Fehlt die Transformation (Statusmeldung im _Displays_-Panel)? Passt die QoS (_Topic → Reliability Policy_)?

📝 Welche Einstellungen waren nötig, damit der Scan erscheint? Fügen Sie einen Screenshot ins Protokoll ein.

*Kamera mit eigenem Skript (Kür, nur wenn Zeit bleibt):* Laden Sie das Skript [view_camera.py](https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/exercises/exercise_03_line_following/templates/view_camera.py) herunter (`wget` + Link). Es abonniert `/image_raw`, Ihr Roboter sendet aber auf `/camera/image_raw`. Starten Sie es trotzdem, **ohne den Code zu ändern** (VL 2, Remapping):

<details>
<summary>Lösung anzeigen</summary>

```bash
python3 view_camera.py --ros-args -r /image_raw:=/camera/image_raw
```

</details>

## D: Eine Fahrt aufzeichnen

    --{{0}}--
Zum Schluss sammeln Sie Daten für Block 2. Sie fahren eine Runde durch die Verkehrswelt und zeichnen dabei Kamera, LiDAR und Odometrie auf. Bevor Sie aufnehmen, schätzen Sie ab, wie groß die Aufnahme wird.

Ab Block 2 entwickeln Sie Ihre Spurerkennung am Schreibtisch — mit Daten, die Sie heute aufnehmen. (Ü 2)

**D.1 Abschätzen, bevor Sie aufnehmen**

```bash
ros2 topic bw /camera/image_raw
ros2 topic bw /scan
```

📝 Wie viele Megabyte pro Sekunde erzeugen Kamera und LiDAR (`ros2 topic bw`, Ü 2)? Wie groß wird eine Aufnahme von 60 Sekunden? Reicht der freie Platz auf Ihrem Laptop?

> **Richtwert:** Unkomprimierte Kamerabilder erzeugen schnell mehrere zehn Megabyte pro Sekunde — eine Minute kann über 1 GB groß werden, und über WLAN gehen dabei Bilder verloren. Nehmen Sie deshalb **30 bis 60 Sekunden** auf. Ist der Platz knapp, lassen Sie die Kamera beim ersten Versuch weg und nehmen sie in einem zweiten, kürzeren Durchgang auf.

**D.2 Aufnehmen**

Prüfen Sie zuerst mit `ros2 topic list`, ob Ihr Roboter auch `/camera/camera_info` sendet — darin steht die Kalibrierung der Kamera, die Sie in P 2 brauchen. Fahren Sie dann langsam (≤ 0,1 m/s) eine Runde entlang der Fahrspur und nehmen Sie auf:

```bash
ros2 bag record -o fahrt_spur --topics /camera/image_raw /camera/camera_info /scan /odom /tf /tf_static
```

Warum `/tf` und `/tf_static`? Ohne sie weiß man beim Abspielen nicht, wo der LiDAR am Roboter sitzt (VL 4).

**Gütekriterium:** Die Fahrspur ist im Kamerabild die meiste Zeit zu sehen, und `ros2 bag info` zeigt für die Kamera annähernd so viele Bilder, wie die Rate aus A.3 erwarten lässt. Fehlen viele Bilder, nehmen Sie kürzer oder ohne LiDAR erneut auf.

**D.3 Prüfen und ohne Roboter abspielen**

```bash
ros2 bag info fahrt_spur
```

📝 Dauer, Anzahl der Nachrichten pro Topic und Größe der Aufnahme?

Erst **nach der Abnahme**: Trennen Sie Ihren Laptop vom Roboter-WLAN und spielen Sie die Aufnahme ab (das können Sie auch zu Hause machen):

```bash
ros2 bag play fahrt_spur --loop
```

Starten Sie RViz und `rqt` (Image View) erneut. 📝 Sehen Sie Scan und Kamerabild wie bei der echten Fahrt?

> **Bewahren Sie die Aufnahme auf!** Sie brauchen sie in Ü 3 und P 2. Große Dateien gehören nicht ins Git-Repository — legen Sie sie auf einem gemeinsamen Speicher Ihrer Gruppe ab und notieren Sie im Protokoll, wo.

## Abnahme

    --{{0}}--
Die Praktika sind Ihre Prüfungsleistung. Zum Ende zeigen Sie den Betreuern, dass Ihre Gruppe die folgenden Punkte erreicht hat.

Zeigen Sie den Betreuern am Ende:

<!-- data-type="none" -->
| Nr. | Nachweis                                                                                     | Aus Teil |
| --- | -------------------------------------------------------------------------------------------- | -------- |
| 1   | `ros2 topic list` auf Ihrem Laptop zeigt die Topics des Roboters (Zwischenabnahme)           | A        |
| 2   | Im Protokoll: Typ von `/scan` und `/cmd_vel` **mit dem Befehl, mit dem er ermittelt wurde**; Reliability des LiDAR **und was daraus für RViz folgt** | B |
| 3   | `tf2_echo` zeigt die Lage des LiDAR relativ zu `base_footprint` (Sollwert $z \approx 0{,}18$ m); die PDF aus `view_frames` liegt im Protokoll (Zwischenabnahme) | B |
| 4   | Der vorbereitete Stoppbefehl hält den fahrenden Roboter an                                   | C        |
| 5   | Scan in RViz und Kamerabild in `rqt` sind sichtbar; die nötigen RViz-Einstellungen stehen im Protokoll | C |
| 6   | `ros2 bag info` zeigt eine Aufnahme von 30–60 s mit Kamera, LiDAR, Odometrie und tf, die das Gütekriterium erfüllt | D |
| 7   | **Verständnisfrage** an ein von den Betreuern gewähltes Gruppenmitglied, z.B. „Warum erscheint der Scan in RViz erst mit Best Effort?“ | alle |

Das Abspielen ohne Roboter (D.3) und das Hochladen des Protokolls erledigen Sie bis zu Beginn von P 2. Fällt der Roboter durch einen Defekt aus, den Ihre Gruppe nicht verschuldet hat (Akku, WLAN), holen Sie die fehlenden Punkte zu Beginn von P 2 nach.

**Zum Schluss**

- [ ] Roboter über die Betreuer herunterfahren lassen bzw. ausschalten, Akku zum Laden abgeben
- [ ] Auf dem eigenen Laptop für zu Hause wieder `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` aktivieren (Zeile in `~/.bashrc` wieder einkommentieren)
- [ ] Aufnahme auf den Gruppenspeicher kopieren

**Ausblick:** In Block 2 lernen Sie die Sensoren genauer kennen — und in **P 2** korrigieren Sie das Kamerabild Ihrer Aufnahme, bevor Sie in **P 3** die Fahrspur darin erkennen.
