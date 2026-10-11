<!--

author:   Sebastian Zug & Claude Code (Opus 5.5) mit LiaScript Teaching-Agent
email:    sebastian.zug@informatik.tu-freiberg.de
version:  0.2.0
language: de
narrator: Deutsch Female

import:   https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/config.md

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/exercises/exercise_00b_ros2_werkzeuge/README.md)

# Übung: Werkzeuge von ROS 2

<!-- data-type="none" -->
| Parameter            | Kursinformationen                                                                                                              |
| -------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| **Veranstaltung:**   | @config.lecture                                                                                                                |
| **Semester**         | @config.semester                                                                                                               |
| **Hochschule:**      | `Technische Universität Freiberg`                                                                                              |
| **Inhalte:**         | `Services, Parameter und Actions mit der Kommandozeile, Daten aufzeichnen und abspielen mit ros2 bag`                           |
| **Link auf GitHub:** | https://github.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/blob/master/exercises/exercise_00b_ros2_werkzeuge/README.md   |
| **Autoren**          | @author                                                                                                                        |

--------------------------------------------------------------------------------

**Voraussetzung:** ROS 2 Jazzy ist auf Ihrem Rechner installiert und der Selbsttest der [Installationsanleitung](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/exercises/installation_ros2/README.md) läuft.

**Zielstellung der heutigen Übung**

In VL 2 haben Sie Topics kennengelernt, in VL 3 eigene Pakete gebaut. Heute probieren Sie die übrigen Kommunikationsmuster von ROS 2 aus und lernen, Daten aufzuzeichnen. Am Ende können Sie

+ für eine Aufgabe des Roboters das passende Kommunikationsmuster begründet auswählen,
+ Services finden, ihren Aufbau lesen und sie aufrufen,
+ Parameter eines Knotens lesen, ändern und in Dateien speichern,
+ Actions starten, ihren Fortschritt verfolgen und sie abbrechen,
+ Nachrichten mit `ros2 bag` aufzeichnen, untersuchen und abspielen.

Für alle Muster gilt dasselbe Vorgehen: **anzeigen – Aufbau lesen – benutzen**. Der Spickzettel am Ende fasst es zusammen.

Alles üben wir am `turtlesim` — dieselben Befehle verwenden Sie später am TurtleBot.

**Ablauf (90 min)**

<!-- data-type="none" -->
| Zeit   | Inhalt                                                           |
| ------ | ---------------------------------------------------------------- |
| 10 min | Leitfrage, Selbsttest der Installation, Hilfe bei Problemen       |
| 5 min  | Überblick: Was bietet ein Knoten an?                              |
| 15 min | Services (S1)                                                    |
| 10 min | Parameter (P1)                                                   |
| 15 min | Actions (A0, A1)                                                 |
| 25 min | Daten aufzeichnen und abspielen (B1)                             |
| 5 min  | Selbsttest, Ausblick                                             |
| 5 min  | Skizzen aus VL 3 (Aufgabe 3) in der Gruppe vergleichen           |

**Kür** für schnelle Gruppen oder zu Hause: Neustart mit Parameterdatei (P1, zweiter Teil), Zusatzaufgabe B2, Abschlussaufgabe, eigener Knoten (Kür K).

--------------------------------------------------------------------------------

## Start: Läuft alles?

**Leitfrage für heute:** Der TurtleBot steht in der Verkehrswelt an einer roten Ampel.

1. Wie erfährt er laufend, welche Farbe die Ampel zeigt?
2. Wie stellen Sie seine Höchstgeschwindigkeit ein, ohne den Code neu zu bauen?
3. Wie schicken Sie ihn in die Parklücke — und erfahren unterwegs, wie weit er ist?

Notieren Sie für jede Frage, welches Kommunikationsmuster aus VL 2 Sie vermuten (Topic, Service, Parameter oder Action). Am Ende der Übung lösen wir auf.

Führen Sie die drei Tests aus der Installationsanleitung aus: `ros2 doctor`, Talker/Listener und das turtlesim-Fenster. Klappt etwas nicht, melden Sie sich jetzt — oder arbeiten Sie heute am Pool-Rechner und lösen das Problem nach der Übung.

**Vier Terminals:** Heute arbeiten Sie mit vier Terminals gleichzeitig. Ordnen Sie sie nebeneinander an:

<!-- data-type="none" -->
| Terminal | Aufgabe                                                                                   |
| :------- | :---------------------------------------------------------------------------------------- |
| **T1**   | `turtlesim_node` — läuft die ganze Übung                                                  |
| **T2**   | Befehle: `ros2 node`, `service`, `param`, `action`, `topic echo`                         |
| **T3**   | `turtle_teleop_key` — **dieses Fenster anklicken**, bevor Sie Tasten drücken              |
| **T4**   | Aufzeichnung mit `ros2 bag`                                                               |

Ubuntu: neues Terminal mit `Ctrl+Alt+T`, neuer Tab mit `Ctrl+Shift+T`. Windows Terminal (WSL2): neuer Tab mit dem Ubuntu-Profil, geteilter Bereich mit `Alt+Shift+D`.

**So lesen Sie die Kästen:** Zeilen mit `>` tippen Sie ein (ohne `>`), alle übrigen Zeilen sind Ausgaben.

## Was bietet ein Knoten an?

Starten Sie in **T1** den Simulator und lassen Sie ihn während der ganzen Übung laufen:

```bash
ros2 run turtlesim turtlesim_node
```

In **T2** fragen Sie ihn nach seinen Schnittstellen:

```text
> ros2 node info /turtlesim
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    ...
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:

```

Alle vier Kommunikationsmuster aus VL 2 tauchen auf: Topics (Subscribers/Publishers), Services, Actions — und die Parameter, die intern ebenfalls über Services (`/turtlesim/…_parameters`) abgefragt werden. Die Einträge folgen immer dem Schema **Name: Typ**.

## Services

Ein Service funktioniert wie ein Funktionsaufruf über das Netz: Ein Knoten stellt eine Anfrage (_Request_), ein anderer bearbeitet sie und schickt **genau eine** Antwort (_Response_) zurück. Die Werkzeuge:

<!-- data-type="none" -->
| Befehl                                   | Wofür?                                      |
| :--------------------------------------- | :------------------------------------------ |
| `ros2 service list -t`                   | alle Services mit Typ                       |
| `ros2 interface show <typ>`              | Aufbau von Anfrage und Antwort              |
| `ros2 service call <name> <typ> "<daten>"` | Service aufrufen                          |

Wie sieht die Anfrage an `/spawn` aus? Die Typbeschreibung enthält zwei Teile, getrennt durch `---`: oben die Anfrage, unten die Antwort.

```text
> ros2 interface show turtlesim/srv/Spawn
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```

Die Daten für den Aufruf schreiben Sie im YAML-Format wie bei `ros2 topic pub` in VL 2: `{feld: wert, feld: wert}`.

```text
> ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
waiting for service to become available...
requester: making request: turtlesim.srv.Spawn_Request(x=2.0, y=2.0, theta=0.0, name='turtle2')

response:
turtlesim.srv.Spawn_Response(name='turtle2')
```

Im Fenster erscheint eine zweite Schildkröte. Sie hat eigene Topics und Services — prüfen Sie es mit `ros2 topic list`.

> **Aufgabe S1:** Lassen Sie `turtle1` mit einem roten Stift der Breite 5 zeichnen und versetzen Sie sie anschließend mit einem Sprung an die Position (8, 8). Fahren Sie dann per Tastatur (`ros2 run turtlesim turtle_teleop_key` in **T3**) ein Stück, um den roten Strich zu sehen. Welche beiden Services brauchen Sie? Schauen Sie sich jeweils zuerst den Typ an.
>
> Zwei Hinweise: Winkel werden in ROS im Bogenmaß angegeben (`1.57` rad ≈ 90°). Und manche Wörter haben in YAML eine Sonderbedeutung — `off`, `on`, `yes`, `no` werden als Wahrheitswerte gelesen. Setzen Sie solche Feldnamen in Anführungszeichen.

<details>
<summary>Lösung anzeigen</summary>

```text
> ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 5, 'off': 0}"
...
response:
turtlesim.srv.SetPen_Response()
> ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 8.0, y: 8.0, theta: 1.57}"
...
response:
turtlesim.srv.TeleportAbsolute_Response()
```

Das Feld `off` steht in Anführungszeichen, weil `off` in YAML sonst als Wahrheitswert „falsch“ gelesen wird. Die Antworten sind leer — beide Services melden nur, dass sie fertig sind. Den Bildschirm löschen Sie mit `ros2 service call /clear std_srvs/srv/Empty`.

</details>

## Parameter

Parameter sind Einstellungen eines Knotens, die sich ohne neues Kompilieren ändern lassen — beim Start oder während des Betriebs. Im Projekt werden das etwa Schwellwerte Ihrer Spurerkennung oder die Höchstgeschwindigkeit sein.

```text
> ros2 param list /turtlesim
  background_b
  background_g
  background_r
  holonomic
  qos_overrides./parameter_events.publisher.depth
  ...
  use_sim_time
> ros2 param get /turtlesim background_b
Integer value is: 255
> ros2 param describe /turtlesim background_r
Parameter name: background_r
  Type: integer
  Description: Red channel of the background color
  Constraints:
    Min value: 0
    Max value: 255
    Step: 1
> ros2 param set /turtlesim background_r 150
Set parameter successful
```

Der Hintergrund färbt sich sofort um. Der Knoten kann Werte auch ablehnen:

```text
> ros2 param set /turtlesim background_r 300
Setting parameter failed: Parameter {background_r} doesn't comply with integer range.
```

Alle aktuellen Werte lassen sich als YAML-Datei sichern:

```text
> ros2 param dump /turtlesim > meine_turtle.yaml
> cat meine_turtle.yaml
/turtlesim:
  ros__parameters:
    background_b: 255
    background_g: 86
    background_r: 150
    holonomic: false
    ...
```

Eine solche Datei lässt sich auf zwei Wegen wieder verwenden — in einen laufenden Knoten laden oder beim Start übergeben:

```bash
ros2 param load /turtlesim meine_turtle.yaml                              # in den laufenden Knoten
ros2 run turtlesim turtlesim_node --ros-args --params-file meine_turtle.yaml   # beim Start
```

Einzelne Werte können Sie beim Start auch direkt angeben — mit `-p name:=wert`, so wie Sie mit `-r` umbenennen:

```bash
ros2 run turtlesim turtlesim_node --ros-args -p background_r:=0
```

Die Datei wird im aktuellen Ordner des jeweiligen Terminals gesucht. Eine neue Datei legen Sie mit einem Editor an, z.B. `nano schwarz.yaml` (speichern mit `Ctrl+O`, beenden mit `Ctrl+X`) oder `code schwarz.yaml` in VS Code.

> **Aufgabe P1:** Erstellen Sie eine Datei `schwarz.yaml`, mit der der Simulator mit schwarzem Hintergrund startet — wie der Boden der Verkehrswelt. Laden Sie sie einmal in den laufenden Knoten und starten Sie den Simulator einmal neu damit.

<details>
<summary>Lösung anzeigen</summary>

```yaml  schwarz.yaml
/turtlesim:
  ros__parameters:
    background_r: 0
    background_g: 0
    background_b: 0
```

```text
> ros2 param load /turtlesim schwarz.yaml
Set parameter background_r successful
Set parameter background_g successful
Set parameter background_b successful
```

Beim Start (nach `Ctrl+C` in T1, im Ordner mit der Datei):

```bash
ros2 run turtlesim turtlesim_node --ros-args --params-file schwarz.yaml
```

In VL 3 haben Sie Launch-Dateien kennengelernt: Dort übergeben Sie dieselbe Datei mit `parameters=['schwarz.yaml']` an `Node(...)`.

</details>

## Actions

Eine Action ist ein **lang laufender Auftrag**. Anders als ein Service liefert sie unterwegs Rückmeldungen (_Feedback_) und lässt sich abbrechen. Sie besteht aus drei Teilen: Ziel (_Goal_), Rückmeldung (_Feedback_) und Ergebnis (_Result_).

```text
> ros2 action list -t
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
> ros2 interface show turtlesim/action/RotateAbsolute
# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining
```

Die drei Blöcke sind Ziel, Ergebnis und Rückmeldung: Wir geben eine Zielrichtung `theta` vor, unterwegs meldet der Knoten den verbleibenden Winkel `remaining`, am Ende den gedrehten Winkel `delta`.

```text
> ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback
Sending goal:
     theta: -1.57

Goal accepted with ID: 512213b8ea4f4f5493b018416b2ddc0b

Feedback:
    remaining: -3.140000104904175

Feedback:
    remaining: -3.124000072479248
...
```

> **Aufgabe A0:** Drehen Sie `turtle1` mit `ros2 action send_goal` nach Westen (`theta: 3.14`) und lassen Sie sich die Rückmeldungen anzeigen. Wie entwickelt sich `remaining`, und welchen Status meldet der Client am Ende?

<details>
<summary>Lösung anzeigen</summary>

```text
> ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 3.14}" --feedback
...
Feedback:
    remaining: 1.9759999513626099
...
Result:
    delta: -2.063999891281128

Goal finished with status: SUCCEEDED
```

`remaining` läuft gegen 0, das Ergebnis `delta` gibt den tatsächlich gedrehten Winkel an. Die genauen Zahlen hängen davon ab, wie die Schildkröte vorher stand. Mit `Ctrl+C` im Client lässt sich ein laufender Auftrag abbrechen; die Kommandozeile meldet dann `Canceling goal...` und eventuell eine technische Warnung, die Sie ignorieren können. Sauberer geht es mit der Tastatur — siehe A1.

</details>

Auch `turtle_teleop_key` ist ein Action-Client: Mit den Buchstabentasten rund um `f` (`g`, `b`, `v`, `c`, `d`, `e`, `r`, `t`) dreht sich die Schildkröte in eine feste Richtung, `f` bricht die Drehung ab. Das Programm zeigt die Belegung beim Start an.

> **Aufgabe A1:** Starten Sie eine Drehung per Tastatur (z.B. `g`) und brechen Sie sie mit `f` ab, bevor sie fertig ist. Was meldet T1? Was passiert, wenn Sie während einer laufenden Drehung eine zweite Taste drücken?

<details>
<summary>Lösung anzeigen</summary>

Beim Abbruch mit `f` meldet der Simulator in T1:

```text
[INFO] [...] [turtlesim]: Rotation goal canceled
```

Eine zweite Taste während einer Drehung schickt ein neues Ziel. Der Simulator bearbeitet immer nur einen Auftrag und verwirft den alten:

```text
[WARN] [...] [turtlesim]: Rotation goal received before a previous goal finished. Aborting previous goal
[INFO] [...] [turtlesim]: Rotation goal completed successfully
```

Der Client des verworfenen Auftrags erhält den Status `ABORTED`, der neue endet mit `SUCCEEDED`. Abbrechen (_cancel_) geht also vom Auftraggeber aus, Verwerfen (_abort_) vom ausführenden Knoten.

Die Kennzeichnungen `INFO` und `WARN` sind die Log-Stufen aus VL 3. Alle Logmeldungen des Systems laufen auch über das Topic `/rosout` — mit `ros2 topic echo /rosout` oder dem Werkzeug `rqt_console` sehen Sie sie gesammelt.

</details>

**Kurze Kontrolle:** Welches Kommunikationsmuster passt?

[ [Topic] [Service] [Parameter] [Action] ]
[  ( )      (X)       ( )         ( )    ] Eine zweite Schildkröte erzeugen
[  ( )      ( )       ( )         (X)    ] Den TurtleBot zur Parklücke fahren lassen und den Fortschritt anzeigen
[  ( )      ( )       (X)         ( )    ] Die Farbe des Hintergrunds festlegen
[  (X)      ( )       ( )         ( )    ] Laufend die Position der Schildkröte melden

## Daten aufzeichnen und abspielen

Mit `ros2 bag` zeichnen Sie Nachrichten ausgewählter Topics in einer Datei auf und spielen sie später wieder ab. Für das Projekt ist das zentral: Sie nehmen einmal eine Fahrt durch die Verkehrswelt auf und entwickeln Ihre Spurerkennung danach am Schreibtisch — immer mit denselben Bildern.

**Schritt 1: Aufzeichnen.** Starten Sie in **T4** die Aufnahme der Fahrbefehle und der Position:

```bash
ros2 bag record -o fahrt --topics /turtle1/cmd_vel /turtle1/pose
```

Fahren Sie die Schildkröte jetzt etwa 10 Sekunden lang mit der Tastatur (T3) und beenden Sie die Aufnahme dann mit `Ctrl+C` in T4. Notieren Sie sofort die Endposition — Sie brauchen sie in Aufgabe B1:

```bash
ros2 topic echo --once /turtle1/pose       # in T2
```

> Existiert der Ordner `fahrt` schon, bricht `ros2 bag record` ab. Wählen Sie dann einen neuen Namen (`-o fahrt2`) oder löschen Sie den alten Ordner (`rm -r fahrt`).

**Schritt 2: Untersuchen.** Die Aufnahme ist ein Ordner mit einer Datei im Format MCAP. Ihre Zahlen weichen ab, je nachdem, wie lange Sie gefahren sind:

```text
> ros2 bag info fahrt
Files:             fahrt_0.mcap
Bag size:          36.6 KiB
Storage id:        mcap
ROS Distro:        jazzy
Duration:          6.464337911s
Messages:          410
Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 5 | Serialization Format: cdr
                   Topic: /turtle1/pose | Type: turtlesim/msg/Pose | Count: 405 | Serialization Format: cdr
```

**Kurze Kontrolle:** In der Aufnahme oben stehen 5 Fahrbefehle, aber 405 Positionen. Warum so viele Positionen?

[( )] Jeder Fahrbefehl wird intern 81-mal wiederholt.
[(X)] Der Simulator veröffentlicht seine Position fortlaufend mit fester Rate, auch im Stillstand — Fahrbefehle kommen nur, wenn eine Taste gedrückt wird.
[( )] `ros2 bag` speichert Positionen mehrfach, um Datenverlust vorzubeugen.
***
Prüfen Sie es: `ros2 topic hz /turtle1/pose` zeigt etwa 62 Nachrichten pro Sekunde. Sensoren verhalten sich genauso — eine Kamera sendet 30 Bilder pro Sekunde, ob sich etwas bewegt oder nicht. Darum werden Aufnahmen schnell groß.
***

**Schritt 3: Abspielen.** Setzen Sie die Simulation zurück und spielen Sie **nur die Fahrbefehle** ab:

```bash
ros2 service call /reset std_srvs/srv/Empty          # in T2
ros2 bag play fahrt --topics /turtle1/cmd_vel        # in T4
```

Die Schildkröte fährt die Strecke ein zweites Mal — ohne dass jemand eine Taste drückt.

> **Aufgabe B1:** Lesen Sie nach dem Abspielen die Endposition mit `ros2 topic echo --once /turtle1/pose` ab und vergleichen Sie sie mit der notierten Position aus Schritt 1. Landet die Schildkröte exakt am selben Ort? Wiederholen Sie `reset` und `play` zwei-, dreimal.

<details>
<summary>Lösung anzeigen</summary>

In unserem Test endete die aufgenommene Fahrt bei x = 4,80, y = 9,41, die abgespielte bei x = 5,78, y = 9,53 — und jede Wiedergabe weicht etwas anders ab. Die Fahrbefehle kommen beim Abspielen nicht auf die Millisekunde genau zur selben Zeit an, und turtlesim führt jeden Befehl für eine feste Zeitspanne aus. Kleine Zeitunterschiede summieren sich über die Fahrt.

Das ist eine wichtige Einsicht für das Projekt: **Befehle blind abzuspielen ist eine Steuerung ohne Rückkopplung.** Ein Roboter, der zuverlässig einer Spur folgen soll, muss laufend messen, wo er ist, und nachregeln. Genau darum geht es in Block 5 (Regelung).

</details>

> **Zusatzaufgabe B2** (wenn Zeit bleibt)**:** Lassen Sie in T2 `ros2 topic echo /turtle1/pose` laufen und spielen Sie in T4 diesmal **nur** `/turtle1/pose` ab (`--topics /turtle1/pose`). Bewegt sich die Schildkröte im Fenster? Warum nicht — und wofür könnte so eine Wiedergabe trotzdem nützlich sein?

<details>
<summary>Lösung anzeigen</summary>

Die Schildkröte bewegt sich nicht: Der Simulator **veröffentlicht** `/turtle1/pose`, er abonniert es nicht. Beim Abspielen gibt es plötzlich zwei Quellen für dasselbe Topic — den Simulator und die Aufnahme. `ros2 topic echo` zeigt beide durcheinander.

Nützlich ist das, wenn der Simulator **nicht** läuft: Dann liefert die Aufnahme die Positionen, und Sie können einen Knoten testen, der Positionen auswertet. Genauso arbeiten Sie später mit Kamerabildern ohne Roboter. Ein häufiger Fehler ist, die Aufnahme abzuspielen, während der echte Sensor noch läuft.

</details>

Wie viele Daten ein Topic pro Sekunde erzeugt, misst `ros2 topic bw /turtle1/pose` — so schätzen Sie vor einer Aufnahme ab, wie groß sie wird.

Weitere nützliche Optionen: `ros2 bag play fahrt --rate 2.0` spielt doppelt so schnell ab, `--loop` wiederholt die Aufnahme, und `ros2 bag record -a` nimmt **alle** Topics auf — beim Roboter mit Kamera ist das selten eine gute Idee.

## Selbsttest und Ausblick

**Auflösung der Leitfrage:** Die Ampelfarbe kommt laufend über ein **Topic**. Die Höchstgeschwindigkeit ist ein **Parameter** des Fahrknotens. Die Fahrt in die Parklücke ist eine **Action** mit Fortschrittsmeldungen. Ein **Service** passt zu einmaligen Anfragen, etwa „Kalibrierung neu laden“.

**Welcher Befehl passt?**

`ros2` [[ (service call) | param set | action send_goal | bag record ]] `/turtle1/teleport_absolute …` versetzt die Schildkröte.

`ros2` [[ service call | (param dump) | topic echo | bag info ]] `/turtlesim > datei.yaml` sichert alle Einstellungen.

`ros2` [[ topic pub | service call | (action send_goal) | param load ]] `… --feedback` startet einen Auftrag und zeigt den Fortschritt.

**Spickzettel**

<!-- data-type="none" -->
| Muster    | Anzeigen                    | Aufbau lesen                   | Benutzen                                   |
| :-------- | :-------------------------- | :----------------------------- | :----------------------------------------- |
| Topic     | `ros2 topic list -t`        | `ros2 interface show <typ>`    | `ros2 topic echo` / `ros2 topic pub`       |
| Service   | `ros2 service list -t`      | `ros2 interface show <typ>`    | `ros2 service call <name> <typ> "<daten>"` |
| Parameter | `ros2 param list <knoten>`  | `ros2 param describe <knoten> <name>` | `ros2 param get` / `set` / `dump` / `load` |
| Action    | `ros2 action list -t`       | `ros2 interface show <typ>`    | `ros2 action send_goal … --feedback`       |
| Aufnahme  | `ros2 bag info <ordner>`    | —                              | `ros2 bag record -o <ordner> --topics <topics>` / `ros2 bag play <ordner>` |

**Abschlussaufgabe** (Kür oder zu Hause, ca. 10 min): Erzeugen Sie `turtle2` bei (5, 5) mit grünem Stift, drehen Sie sie per Action nach Norden (`theta: 1.57`) und zeichnen Sie 10 Sekunden lang nur ihre Position auf, während Sie sie fahren. Notieren Sie Ihre Befehlsfolge. Tipp: Die Tastatursteuerung sendet an `turtle1` — lenken Sie sie per Remapping (VL 2) auf `turtle2` um.

**Prüfkriterium:** `ros2 bag info` zeigt nur das Topic `/turtle2/pose`, und im Fenster ist eine grüne Linie zu sehen, sobald `turtle2` fährt.

<details>
<summary>Hinweis zur Tastatursteuerung anzeigen</summary>

```bash
ros2 run turtlesim turtle_teleop_key --ros-args -r turtle1/cmd_vel:=turtle2/cmd_vel
```

</details>

**Kür K — ein eigener Knoten** (zu Hause, verbindet VL 3 mit heute): Legen Sie im Workspace aus VL 3 ein Python-Paket `turtle_watcher` an. Sein Knoten abonniert `/turtle1/pose` und gibt mit dem Logger eine Warnung aus, sobald die Schildkröte dem Rand näher als 1 kommt (das Fenster reicht von 0 bis etwa 11). Starten Sie Simulator und Knoten gemeinsam mit einer Launch-Datei, die den Simulator mit `schwarz.yaml` lädt (`parameters=['schwarz.yaml']`). **Prüfkriterium:** Fahren Sie an den Rand — im Terminal erscheint `[WARN]`.

**Ausblick:** In VL 4 geht es um Koordinatensysteme — die Position `x`, `y`, `theta` der Schildkröte ist schon eine solche Angabe — und um Quality of Service. Im Praktikum P 1 wenden Sie alles Gelernte am echten TurtleBot an: verbinden, fahren und eine Fahrt mit LiDAR- und Kameradaten aufzeichnen.
