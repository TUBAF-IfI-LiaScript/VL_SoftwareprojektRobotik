<!--

author:   Sebastian Zug & Georg Jäger & Claude Code (Opus 5.5) mit LiaScript Teaching-Agent
email:    sebastian.zug@informatik.tu-freiberg.de & Georg.Jaeger@informatik.tu-freiberg.de
version:  0.1.0
language: de
narrator: Deutsch Female

import:   https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/config.md
          https://github.com/liascript/CodeRunner

@bash:      @LIA.eval(`["main.sh"]`, `none`, `bash main.sh`)
@bashShell: @LIA.eval(`["main.sh"]`, `none`, `bash --rcfile main.sh -i`)
@bashLog:   @LIA.eval(`["main.sh", "ros.log"]`, `none`, `bash main.sh`)

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/exercises/exercise_00_linux_shell/README.md)

# Übung: OS + Linux Shell

<!-- data-type="none" -->
| Parameter            | Kursinformationen                                                                                                       |
| -------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| **Veranstaltung:**   | @config.lecture                                                                                                         |
| **Semester**         | @config.semester                                                                                                        |
| **Hochschule:**      | `Technische Universität Freiberg`                                                                                       |
| **Inhalte:**         | `Arbeiten im Linux-Terminal: Dateisystem, Rechte, Prozesse, Umgebungsvariablen`                                         |
| **Link auf GitHub:** | https://github.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/blob/master/exercises/exercise_00_linux_shell/README.md |
| **Autoren**          | @author                                                                                                                 |

--------------------------------------------------------------------------------

**Zielstellung der heutigen Übung**

+ Sich im Linux-Dateisystem mit absoluten und relativen Pfaden bewegen
+ Dateien anlegen, kopieren, durchsuchen und Ausgaben mit Pipes verknüpfen
+ Dateirechte lesen und setzen, eigene Skripte ausführbar machen
+ Prozesse starten, beobachten und beenden
+ Umgebungsvariablen verstehen — und erklären, warum `source /opt/ros/jazzy/setup.bash` nötig ist

**Ablauf (90 min)**

<!-- data-type="none" -->
| Zeit      | Inhalt                                                     |
| --------- | ---------------------------------------------------------- |
| 10 min    | Warum Terminal? Arbeitsumgebung                            |
| 30 min    | Grundlagen mit interaktiven Beispielen                     |
| 40 min    | Aufgaben 1–4 am eigenen Ubuntu-System                      |
| 10 min    | Selbsttest und Ausblick auf die ROS-2-Installation         |

--------------------------------------------------------------------------------

## Warum Terminal?

    --{{0}}--
In der Robotik kommen Sie am Terminal nicht vorbei. Der TurtleBot hat keinen Bildschirm, Sie erreichen ihn über das Netzwerk mit `ssh`. ROS 2 wird über Kommandozeilenwerkzeuge gestartet, konfiguriert und untersucht. Und viele Fehler, die Ihnen in den nächsten Wochen begegnen, sind in Wahrheit keine ROS-Fehler, sondern Fehler in der Shell-Umgebung: eine vergessene Umgebungsvariable, ein falscher Pfad, fehlende Rechte.

```ascii
  +-------------------+          ssh / ROS 2 (DDS)          +---------------------+
  | Ihr Laptop        |  <------------------------------->  | TurtleBot 3         |
  | Terminal 1: rviz2 |                                     | Raspberry Pi        |
  | Terminal 2: Ihr   |                                     | kein Bildschirm,    |
  |   ROS-Knoten      |                                     | nur Terminal        |
  +-------------------+                                     +---------------------+
```

**Begriffe**

+ **Terminal** — das Fenster, in dem Sie Text eingeben und Ausgaben sehen
+ **Shell** — das Programm, das Ihre Eingaben interpretiert; bei Ubuntu ist das `bash`
+ **Prompt** — die Eingabeaufforderung, z. B. `student@laptop:~/ros2_ws$` (Benutzer, Rechner, aktuelles Verzeichnis)

**Ihre Arbeitsumgebung**

<!-- data-type="none" -->
| Variante                  | Bemerkung                                                                                  |
| ------------------------- | ------------------------------------------------------------------------------------------ |
| Ubuntu 24.04 nativ        | empfohlen — alle Werkzeuge der Veranstaltung laufen direkt                                  |
| WSL2 unter Windows        | gute Alternative, `wsl --install -d Ubuntu-24.04`                                           |
| Rechnerpool               | für den Einstieg, aber ohne eigene Installation von ROS 2                                   |

### Wie funktionieren die Beispiele in diesem Skript?

    --{{0}}--
In diesem Skript gibt es zwei Arten, Befehle auszuprobieren. Die Codeblöcke können Sie bearbeiten und mit dem Ausführen-Knopf auf einem Server starten. Manche Blöcke öffnen danach eine interaktive Shell, in der Sie weitertippen können. Zusätzlich finden Sie unten ein vollständiges Linux, das direkt in Ihrem Browser läuft.

+ **Codeblock mit Ausführen-Knopf:** Der Code läuft auf einem Server mit Ubuntu und `bash`. Sie können ihn verändern und neu starten.
+ **Interaktive Blöcke:** Nach dem Skript erscheint ein Prompt — tippen Sie weitere Befehle ein, `exit` beendet die Shell.
+ **Browser-Linux:** ein kleines Alpine-Linux (JSLinux), das komplett in Ihrem Browser läuft. Es nutzt die Shell `ash` statt `bash`; die Grundbefehle sind identisch.

> **Hinweis:** Die Server-Beispiele laufen jeweils in einem frischen, temporären Verzeichnis. Arbeiten Sie dort mit **relativen Pfaden** — `~` zeigt auf dem Server nicht auf ein eigenes Home-Verzeichnis.

Probieren Sie es aus: Starten Sie den Block und geben Sie danach `whoami` und `date` ein.

```bash  main.sh
echo "Hallo aus der Shell!"
echo "Ich bin gerade in: $(pwd)"
```
@bashShell

## Dateisystem

    --{{0}}--
Linux kennt keine Laufwerksbuchstaben. Alles hängt in einem einzigen Baum, der bei der Wurzel `/` beginnt. Ihr persönliches Verzeichnis liegt unter `/home/benutzername` und hat die Abkürzung `~`. ROS 2 selbst wird später unter `/opt/ros/jazzy` installiert.

```text
/
├── home/
│   └── student/            <- ~  (Ihr Home-Verzeichnis)
│       └── ros2_ws/
│           └── src/
├── opt/
│   └── ros/
│       └── jazzy/          <- ROS 2 Installation
├── etc/                    <- Konfigurationsdateien
├── tmp/                    <- temporäre Dateien
└── usr/bin/                <- Programme
```

<!-- data-type="none" -->
| Befehl         | Bedeutung                                   | Beispiel                       |
| -------------- | ------------------------------------------- | ------------------------------ |
| `pwd`          | aktuelles Verzeichnis anzeigen              | `pwd`                          |
| `ls`           | Inhalt auflisten (`-l` lang, `-a` versteckt)| `ls -la`                       |
| `cd`           | Verzeichnis wechseln                        | `cd ros2_ws`, `cd ..`, `cd ~`  |
| `mkdir`        | Verzeichnis anlegen (`-p` mit Zwischenpfad) | `mkdir -p ros2_ws/src`         |
| `touch`        | leere Datei anlegen                         | `touch notes.txt`              |
| `cp` / `mv`    | kopieren / verschieben bzw. umbenennen      | `cp a.txt b.txt`               |
| `rm`           | löschen (`-r` rekursiv) — **ohne Papierkorb!** | `rm -r build`               |

**Absolute und relative Pfade**

+ **absolut** beginnt bei `/`: `/opt/ros/jazzy/setup.bash`
+ **relativ** geht vom aktuellen Verzeichnis aus: `src/my_package`, `../install`
+ `.` ist das aktuelle, `..` das übergeordnete Verzeichnis

```bash  main.sh
mkdir -p ros2_ws/src/my_package
cd ros2_ws
pwd
touch src/my_package/package.xml
ls -R
cd src/my_package
echo "Jetzt bin ich in: $(pwd)"
ls ../..
```
@bash

    --{{1}}--
Verändern Sie das Beispiel: Legen Sie zusätzlich ein Verzeichnis `build` an und löschen Sie es wieder. Was passiert, wenn Sie `mkdir` ohne die Option `-p` verwenden?

## Dateien ansehen, suchen und verknüpfen

    --{{0}}--
Die eigentliche Stärke der Shell liegt im Kombinieren kleiner Werkzeuge. Jedes Programm liest Text und gibt Text aus. Mit dem senkrechten Strich, der Pipe, leiten Sie die Ausgabe eines Programms als Eingabe an das nächste weiter. Mit dem Größerzeichen schreiben Sie eine Ausgabe in eine Datei.

<!-- data-type="none" -->
| Befehl                  | Bedeutung                                         |
| ----------------------- | ------------------------------------------------- |
| `cat datei`             | Datei vollständig ausgeben                        |
| `less datei`            | Datei seitenweise anzeigen (`q` beendet)          |
| `head -n 5` / `tail -n 5` | erste / letzte Zeilen; `tail -f` folgt einer wachsenden Datei |
| `grep muster datei`     | Zeilen mit einem Suchmuster finden (`-i`, `-c`, `-v`) |
| `find . -name "*.py"`   | Dateien im Verzeichnisbaum suchen                 |
| `wc -l`                 | Zeilen zählen                                     |
| `a \| b`                | Pipe: Ausgabe von `a` wird Eingabe von `b`        |
| `> datei` / `>> datei`  | Ausgabe in Datei schreiben / anhängen             |

Das folgende Beispiel arbeitet mit einem (gekürzten) Log eines ROS-2-Systems:

```bash  main.sh
echo "--- Die ersten 3 Zeilen:"
head -n 3 ros.log
echo "--- Alle Warnungen:"
grep "WARN" ros.log
echo "--- Anzahl der Fehler:"
grep -c "ERROR" ros.log
echo "--- Welche Knoten melden Fehler?"
grep "ERROR" ros.log | cut -d' ' -f3 | sort | uniq -c
grep "ERROR" ros.log > errors.txt
echo "--- errors.txt hat $(wc -l < errors.txt) Zeilen"
```
```text  ros.log
[1728550001.10] [INFO] [camera_node]: Camera started, 640x480 @ 30 fps
[1728550001.20] [INFO] [lidar_node]: RPLIDAR C1 connected on /dev/ttyUSB0
[1728550001.95] [WARN] [lidar_node]: Scan frequency below 10 Hz
[1728550002.31] [INFO] [line_detector]: Subscribed to /camera/image_raw
[1728550003.02] [ERROR] [line_detector]: No line found in image
[1728550003.40] [WARN] [controller_node]: cmd_vel limited to 0.22 m/s
[1728550004.12] [ERROR] [line_detector]: No line found in image
[1728550004.80] [ERROR] [lidar_node]: Lost connection to /dev/ttyUSB0
[1728550005.01] [INFO] [controller_node]: Emergency stop
```
@bashLog

## Rechte und Benutzer

    --{{0}}--
Jede Datei gehört einem Benutzer und einer Gruppe. Für den Besitzer, die Gruppe und alle anderen ist jeweils festgelegt, ob gelesen, geschrieben oder ausgeführt werden darf. Das sehen Sie mit `ls -l`. Ein Skript, das Sie selbst geschrieben haben, dürfen Sie zunächst nicht ausführen — dafür müssen Sie das Ausführungsrecht setzen.

```text
  -rwxr-xr--  1 student student  42 Oct 20 10:00 hello.sh
  |\_/\_/\_/    \_____/ \_____/
  | |  |  |        |       +-- Gruppe
  | |  |  |        +---------- Besitzer
  | |  |  +-- alle anderen:  r--  = 4        lesen
  | |  +----- Gruppe:        r-x  = 4+1 = 5  lesen, ausführen
  | +-------- Besitzer:      rwx  = 4+2+1 = 7
  +---------- Dateityp (- Datei, d Verzeichnis)
```

+ `chmod +x datei` — Ausführungsrecht hinzufügen
+ `chmod 754 datei` — Rechte als Zahl setzen (r = 4, w = 2, x = 1)
+ `sudo befehl` — Befehl mit Administratorrechten ausführen, z. B. `sudo apt install ...`
+ **Shebang:** Die erste Zeile `#!/bin/bash` legt fest, mit welchem Programm ein Skript ausgeführt wird.

```bash  main.sh
cat > hello.sh << 'EOF'
#!/bin/bash
echo "Hallo, ich bin ein Skript!"
EOF
ls -l hello.sh
./hello.sh || echo ">>> Nicht ausführbar!"
chmod +x hello.sh
ls -l hello.sh
./hello.sh
```
@bash

## Prozesse

    --{{0}}--
Jedes laufende Programm ist ein Prozess mit einer eigenen Nummer, der PID. In ROS 2 ist jeder Knoten ein Prozess — Sie werden also oft mehrere Programme parallel laufen lassen, jedes in seinem eigenen Terminal. Ein Programm im Vordergrund beenden Sie mit `Strg+C`.

<!-- data-type="none" -->
| Befehl / Taste       | Bedeutung                                              |
| -------------------- | ------------------------------------------------------ |
| `Strg+C`             | laufendes Programm im Vordergrund beenden              |
| `befehl &`           | Programm im Hintergrund starten                        |
| `ps aux`, `pgrep`    | Prozesse anzeigen / suchen                             |
| `top`, `htop`        | Prozesse und Auslastung live anzeigen (`q` beendet)    |
| `kill PID`           | Prozess beenden                                        |
| `tmux`               | mehrere Terminals in einem Fenster — praktisch per `ssh` |

```bash  main.sh
sleep 30 &
PID=$!
echo "Hintergrundprozess gestartet mit PID $PID"
ps -o pid,cmd -p $PID
kill $PID
sleep 0.5
ps -p $PID > /dev/null || echo "Prozess $PID wurde beendet"
```
@bash

## Umgebungsvariablen

    --{{0}}--
Jetzt kommen wir zum wichtigsten Abschnitt für ROS 2. Jede Shell hat eine Umgebung: eine Sammlung von Variablen wie PATH, HOME oder später ROS_DOMAIN_ID. Programme lesen diese Variablen, um zu wissen, wo sie Dateien finden oder mit wem sie kommunizieren sollen. Entscheidend ist: Die Umgebung gilt nur für die aktuelle Shell und die Programme, die aus ihr heraus gestartet werden. Ein neues Terminal beginnt wieder von vorn.

<!-- data-type="none" -->
| Befehl                       | Bedeutung                                                      |
| ---------------------------- | -------------------------------------------------------------- |
| `env`                        | alle Umgebungsvariablen anzeigen                               |
| `echo $PATH`                 | Wert einer Variable ausgeben                                   |
| `NAME=wert`                  | Shell-Variable setzen — nur in dieser Shell sichtbar           |
| `export NAME=wert`           | Umgebungsvariable setzen — auch für gestartete Programme sichtbar |
| `~/.bashrc`                  | wird beim Öffnen jedes neuen Terminals ausgeführt              |
| `source datei`               | Datei **in der aktuellen Shell** ausführen                     |

### `export` — wer sieht die Variable?

```bash  main.sh
LOKAL="nur hier"
export GLOBAL="auch für Kindprozesse"
echo "In der Shell:   LOKAL=$LOKAL, GLOBAL=$GLOBAL"
bash -c 'echo "Im Kindprozess: LOKAL=$LOKAL, GLOBAL=$GLOBAL"'
```
@bash

    --{{0}}--
Der Kindprozess, hier eine zweite bash, sieht nur die exportierte Variable. Genau so verhält es sich mit einem ROS-Knoten, den Sie aus der Shell starten: Er sieht nur, was exportiert wurde.

### `PATH` — wo findet die Shell Programme?

    --{{0}}--
Wenn Sie einen Befehl wie ros2 eintippen, durchsucht die Shell der Reihe nach alle Verzeichnisse, die in der Variablen PATH stehen. Ist ein Programm dort nicht zu finden, erhalten Sie die Meldung command not found. Genau das passiert, wenn Sie vergessen, ROS 2 zu sourcen.

```bash  main.sh
mkdir -p mybin
printf '#!/bin/bash\necho "Hallo von mytool"\n' > mybin/mytool
chmod +x mybin/mytool
mytool || echo ">>> mytool liegt nicht im PATH"
export PATH="$PWD/mybin:$PATH"
echo "PATH beginnt jetzt mit: ${PATH%%:*}"
mytool
which mytool
```
@bash

### `source` oder ausführen?

    --{{0}}--
Ein Skript, das Sie mit Punkt-Schrägstrich ausführen, läuft in einer neuen Shell. Alle Variablen, die es setzt, verschwinden mit dieser Shell. Mit source dagegen werden die Befehle in Ihrer aktuellen Shell ausgeführt, und die Variablen bleiben erhalten. Deshalb heißt es bei ROS 2 immer source /opt/ros/jazzy/setup.bash und nicht einfach ausführen.

```bash  main.sh
cat > setup.bash << 'EOF'
export ROS_DOMAIN_ID=42
export MY_SETUP_DONE=yes
EOF
chmod +x setup.bash

./setup.bash
echo "Nach ./setup.bash:     ROS_DOMAIN_ID='$ROS_DOMAIN_ID'"

source setup.bash
echo "Nach source setup.bash: ROS_DOMAIN_ID='$ROS_DOMAIN_ID'"
```
@bash

**Was bedeutet das für ROS 2?**

```bash
# in jedem neuen Terminal nötig — oder einmalig in ~/.bashrc eintragen:
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42          # Ihre Gruppennummer, trennt die TurtleBots im Labor
```

## Pakete installieren und Fernzugriff

    --{{0}}--
Zwei Werkzeuge brauchen Sie in den nächsten Wochen ständig: die Paketverwaltung apt, mit der Sie Software installieren, und ssh, mit dem Sie sich auf dem TurtleBot anmelden.

**Paketverwaltung**

```bash
sudo apt update                    # Paketlisten aktualisieren
sudo apt install tree htop tmux    # Pakete installieren
apt search turtlebot3              # Pakete suchen
```

**Fernzugriff auf den TurtleBot**

```bash
ssh ubuntu@192.168.0.42            # anmelden (Adresse bekommen Sie im Praktikum)
scp mein_skript.py ubuntu@192.168.0.42:~/   # Datei kopieren
exit                               # Verbindung beenden
```

> Im Praktikum erhält jede Gruppe Adresse und Zugangsdaten ihres TurtleBots.

## Freies Terminal im Browser

    --{{0}}--
Hier können Sie frei experimentieren. Das Linux läuft vollständig in Ihrem Browser, Sie können nichts kaputt machen. Nach dem Neuladen der Seite ist alles wieder im Ausgangszustand.

??[JSLinux: Alpine Linux im Browser](https://bellard.org/jslinux/vm.html?url=alpine-x86.cfg&mem=192 "JSLinux von Fabrice Bellard — Alpine Linux mit der Shell ash")

**Vorschläge zum Ausprobieren**

+ `cd /` und `ls` — wie sieht der Verzeichnisbaum aus?
+ `cat /etc/os-release` — welches System läuft hier?
+ `echo $PATH` und `env`
+ `ls -l /bin | head` — wem gehören die Programme, welche Rechte haben sie?

## Aufgaben

    --{{0}}--
Die folgenden Aufgaben lösen Sie auf Ihrem eigenen Ubuntu-System oder in WSL2. Genau diese Umgebung brauchen Sie in der nächsten Übung für die Installation von ROS 2.

### Aufgabe 1: Workspace anlegen

Legen Sie in Ihrem Home-Verzeichnis folgende Struktur an — mit möglichst wenigen Befehlen:

```text
~/ros2_ws/
├── src/
│   ├── turtle_tools/
│   │   └── README.md
│   └── line_follower/
└── notes.txt
```

Kopieren Sie anschließend `notes.txt` nach `src/line_follower/` und benennen Sie die Kopie in `todo.txt` um.

**Prüfkriterium:** `find ~/ros2_ws` listet genau die Dateien und Verzeichnisse oben plus `src/line_follower/todo.txt`.

<details>
<summary>**Lösungsvorschlag**</summary>

```bash
mkdir -p ~/ros2_ws/src/turtle_tools ~/ros2_ws/src/line_follower
touch ~/ros2_ws/src/turtle_tools/README.md ~/ros2_ws/notes.txt
cp ~/ros2_ws/notes.txt ~/ros2_ws/src/line_follower/todo.txt
find ~/ros2_ws
```

</details>

### Aufgabe 2: Fehlersuche im Log

Laden Sie das Log aus dem Abschnitt „Dateien ansehen, suchen und verknüpfen“ als `ros.log` in Ihren Workspace (oder kopieren Sie es per Copy & Paste mit `cat > ros.log`).

1. Wie viele Zeilen enthalten `INFO`, `WARN` und `ERROR`?
2. Schreiben Sie alle Meldungen des `lidar_node` in die Datei `lidar.txt`.
3. Welcher Knoten meldet die meisten Fehler?

Wie viele `ERROR`-Zeilen enthält das Log?

[[3]]
[[?]] `grep -c "ERROR" ros.log`
***
`grep -c` zählt die passenden Zeilen. Für Aufgabe 3: `grep "ERROR" ros.log | cut -d' ' -f3 | sort | uniq -c | sort -rn` — der `line_detector` meldet zwei Fehler.
***

### Aufgabe 3: Ein eigenes Werkzeug

1. Schreiben Sie ein Skript `~/bin/rosinfo.sh`, das das aktuelle Datum, den Rechnernamen (`hostname`) und den Wert von `ROS_DOMAIN_ID` ausgibt.
2. Machen Sie es ausführbar.
3. Sorgen Sie dafür, dass Sie es aus jedem Verzeichnis einfach mit `rosinfo.sh` aufrufen können.

**Prüfkriterium:** `cd /tmp && rosinfo.sh` funktioniert — auch in einem neu geöffneten Terminal.

<details>
<summary>**Lösungsvorschlag**</summary>

```bash
mkdir -p ~/bin
cat > ~/bin/rosinfo.sh << 'EOF'
#!/bin/bash
echo "Datum:         $(date)"
echo "Rechner:       $(hostname)"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-nicht gesetzt}"
EOF
chmod +x ~/bin/rosinfo.sh
echo 'export PATH="$HOME/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

Unter Ubuntu wird `~/bin` beim Anmelden automatisch in den `PATH` aufgenommen, sofern das Verzeichnis existiert — der Eintrag in `~/.bashrc` macht das Verhalten aber explizit und sofort wirksam.

</details>

### Aufgabe 4: Zwei Terminals, zwei Welten

1. Öffnen Sie zwei Terminals. Setzen Sie im ersten `export ROS_DOMAIN_ID=7` und prüfen Sie den Wert in beiden Terminals mit `rosinfo.sh`. Erklären Sie das Ergebnis.
2. Sorgen Sie dafür, dass `ROS_DOMAIN_ID` in **jedem** neuen Terminal automatisch auf Ihre Gruppennummer gesetzt ist.
3. Warum reicht es nicht, die Zeile in eine Datei `setup.sh` zu schreiben und diese mit `./setup.sh` auszuführen?

**Prüfkriterium:** Ein neues Terminal zeigt mit `echo $ROS_DOMAIN_ID` sofort Ihre Gruppennummer.

<details>
<summary>**Lösungsvorschlag**</summary>

1. Die Variable existiert nur in der Shell des ersten Terminals und in den daraus gestarteten Programmen. Das zweite Terminal ist eine eigene Shell mit eigener Umgebung.
2. `echo 'export ROS_DOMAIN_ID=7' >> ~/.bashrc` — die `~/.bashrc` wird beim Öffnen jedes neuen Terminals ausgeführt.
3. `./setup.sh` startet eine neue Shell als Kindprozess. Die Variable wird dort gesetzt und verschwindet mit dem Ende des Skripts. Nur `source setup.sh` führt die Befehle in der aktuellen Shell aus.

</details>

## Selbsttest

Welche Rechte hat eine Datei nach `chmod 754 datei`?

[( )] `rwxrwxr--`
[(X)] `rwxr-xr--`
[( )] `rwxr--r-x`
[( )] `r-xr-xr--`
***
7 = 4+2+1 = `rwx` (Besitzer), 5 = 4+1 = `r-x` (Gruppe), 4 = `r--` (alle anderen).
***

Sie befinden sich in `/home/student/ros2_ws/src`. Welcher Befehl führt nach `/home/student/ros2_ws/install`?

[[X]] `cd ../install`
[[X]] `cd /home/student/ros2_ws/install`
[[X]] `cd ~/ros2_ws/install`
[[ ]] `cd /install`
[[ ]] `cd ./install`

In einem neuen Terminal meldet `ros2 topic list` die Fehlermeldung `ros2: command not found`. Was ist die wahrscheinlichste Ursache?

[( )] ROS 2 ist nicht installiert.
[(X)] `/opt/ros/jazzy/setup.bash` wurde in diesem Terminal nicht gesourct.
[( )] Die Datei `ros2` hat keine Leserechte.
[( )] Es läuft kein ROS-Knoten.
***
Nach der Installation liegen die ROS-2-Programme unter `/opt/ros/jazzy`. Dieses Verzeichnis steht erst nach `source /opt/ros/jazzy/setup.bash` im `PATH` — und zwar nur in dem Terminal, in dem Sie das ausgeführt haben, außer Sie tragen es in `~/.bashrc` ein.
***

Ein Skript setzt `export ROS_DOMAIN_ID=5`. Sie starten es mit `./skript.sh` und geben danach `echo $ROS_DOMAIN_ID` ein. Was sehen Sie?

[[ (den vorherigen Wert bzw. eine leere Zeile) | 5 | eine Fehlermeldung ]]

## Ausblick

    --{{0}}--
Mit diesen Grundlagen sind Sie für die nächsten Schritte gerüstet. In der Vorlesung geht es um die Konzepte von ROS 2, in der nächsten Übung installieren Sie ROS 2 Jazzy — und dabei begegnen Ihnen apt, sudo, source und die Datei bashrc wieder.

+ **VL 2** Grundlagen ROS 2 — Architektur, Knoten, Topics, Nachrichten
+ **Ü 2** Installation / Grundlagen ROS 2 — ROS 2 Jazzy einrichten, erste Schritte mit `ros2`

**Weiterführende Ressourcen**

+ [Ubuntu Tutorial: The Linux command line for beginners](https://ubuntu.com/tutorials/command-line-for-beginners)
+ [ROS 2 Jazzy: Configuring environment](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
+ [explainshell.com](https://explainshell.com/) — zerlegt beliebige Shell-Befehle in ihre Bestandteile

!?[Linux Command Line Full Course](https://www.youtube.com/watch?v=2PGnYjbYuUo "Geek's Lesson: Linux Command Line Full course – Beginners to Experts (englisch)")
