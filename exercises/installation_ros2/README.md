<!--

author:   Sebastian Zug & Claude Code (Opus 5.5) mit LiaScript Teaching-Agent
email:    sebastian.zug@informatik.tu-freiberg.de
version:  0.1.0
language: de
narrator: Deutsch Female

import:   https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/config.md

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/exercises/installation_ros2/README.md)

# Installation von ROS 2 Jazzy

<!-- data-type="none" -->
| Parameter            | Kursinformationen                                                                                                     |
| -------------------- | --------------------------------------------------------------------------------------------------------------------- |
| **Veranstaltung:**   | @config.lecture                                                                                                       |
| **Semester**         | @config.semester                                                                                                      |
| **Hochschule:**      | `Technische Universität Freiberg`                                                                                     |
| **Inhalte:**         | `Selbstständige Installation von ROS 2 Jazzy vor der Übung Ü 2`                                                        |
| **Link auf GitHub:** | https://github.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/blob/master/exercises/installation_ros2/README.md     |
| **Autoren**          | @author                                                                                                               |

--------------------------------------------------------------------------------

**Bis zur Übung Ü 2** installieren Sie ROS 2 Jazzy selbstständig nach dieser Anleitung. Rechnen Sie mit 30 bis 60 Minuten, der größte Teil ist Download. In der Übung prüfen wir in den ersten Minuten gemeinsam, ob alles läuft — die restliche Zeit gehört der Arbeit mit ROS 2.

**So lesen Sie die Befehle:** In grauen Kästen ohne `>` steht nur, was Sie eintippen. In Kästen mit Ausgaben stehen Ihre Eingaben hinter `>` — tippen Sie diese **ohne** das `>`, alle anderen Zeilen sind Ausgaben des Rechners. `sudo` fragt nach Ihrem Passwort; die Eingabe bleibt unsichtbar, das ist normal. Fragt `apt` mit `[Y/n]` nach, bestätigen Sie mit Enter.

> Diese Anleitung folgt der offiziellen [Installationsanleitung für Ubuntu](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) und wurde im Oktober 2026 auf einem frischen Ubuntu 24.04 getestet. Bei Abweichungen gilt die offizielle Anleitung.

--------------------------------------------------------------------------------

## Schritt 0: Die richtige Umgebung

ROS 2 Jazzy läuft auf **Ubuntu 24.04** („Noble“). Wählen Sie eine der folgenden Varianten:

<!-- data-type="none" -->
| Variante                     | Hinweise                                                                                        |
| :--------------------------- | :---------------------------------------------------------------------------------------------- |
| Ubuntu 24.04 nativ           | empfohlen; auch als zweites System neben Windows (Dual Boot)                                     |
| Windows 11 mit WSL2          | gut geeignet; grafische Programme laufen über WSLg. Einrichtung siehe unten                      |
| Rechnerpool des Instituts    | ROS 2 Jazzy ist installiert — Sie können diese Anleitung überspringen und direkt den Selbsttest machen |
| macOS, ältere Ubuntu-Versionen | nicht unterstützt. Nutzen Sie den Rechnerpool oder melden Sie sich bei den Betreuern             |

Welche Ubuntu-Version Sie haben, verrät:

```bash
lsb_release -a        # erwartet: Release 24.04, Codename noble
```

Planen Sie rund **10 GB freien Speicherplatz** ein.

**WSL2 einrichten (nur Windows):**

1. Startmenü → „PowerShell“ mit rechter Maustaste → _Als Administrator ausführen_. Dort eingeben:

   ```text
   wsl --install -d Ubuntu-24.04
   ```

2. Windows neu starten. Danach öffnet sich ein Ubuntu-Fenster, in dem Sie einen Benutzernamen und ein Passwort festlegen (das brauchen Sie später für `sudo`).
3. Später öffnen Sie Ubuntu über das Startmenü („Ubuntu 24.04“) oder als Profil im _Windows Terminal_. Ein **zweites Terminal** öffnen Sie dort mit einem neuen Tab, ein geteiltes Fenster mit `Alt+Shift+D`. Tipp: Stellen Sie im Windows Terminal unter _Einstellungen → Standardprofil_ „Ubuntu 24.04“ ein.
4. Prüfen Sie in der PowerShell mit `wsl -l -v`, dass „Ubuntu-24.04“ mit `VERSION 2` aufgeführt ist. Haben Sie schon ein älteres Ubuntu in WSL, bleibt es erhalten — achten Sie darauf, das richtige zu öffnen.

Alle folgenden Befehle geben Sie im Ubuntu-Terminal ein, nicht in der PowerShell.

## Schritt 1: Paketquellen einrichten

Die Befehle kennen Sie aus Ü 1: `sudo` führt einen Befehl mit Administratorrechten aus, `apt` installiert Pakete. Geben Sie die Blöcke nacheinander ein und prüfen Sie jeweils, ob Fehlermeldungen erscheinen.

**Zeichensatz prüfen** — ROS 2 erwartet UTF-8:

```bash
locale        # in der Ausgabe sollte UTF-8 vorkommen
```

Falls nicht:

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

**Universe-Paketquelle aktivieren:**

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

**ROS-2-Paketquelle hinzufügen** — das Paket `ros2-apt-source` trägt die Quelle und ihren Schlüssel ein:

```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

> Die zweite Zeile fragt bei GitHub die aktuelle Version des Pakets ab und speichert sie in einer Umgebungsvariablen — genau das Prinzip aus Ü 1.

## Schritt 2: ROS 2 installieren

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop ros-dev-tools
```

`ros-jazzy-desktop` enthält ROS 2 mit allen Werkzeugen der Vorlesung (`rqt`, RViz, `turtlesim`, Beispiele), `ros-dev-tools` unter anderem das Build-Werkzeug `colcon`. Der Download umfasst mehrere Gigabyte.

## Schritt 3: Umgebung dauerhaft laden

Damit Sie nicht in jedem neuen Terminal `source /opt/ros/jazzy/setup.bash` eingeben müssen, tragen Sie den Befehl in die Datei `~/.bashrc` ein (vgl. Ü 1, Aufgabe 4):

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Ergänzen Sie außerdem diese Zeile:

```bash
echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> ~/.bashrc
```

ROS 2 sucht sonst im ganzen Netz nach anderen Knoten (VL 2, „Discovery“). Im WLAN der Universität würden Ihre Knoten dann die Knoten Ihrer Kommilitonen sehen und umgekehrt. Mit `LOCALHOST` bleibt alles auf Ihrem Rechner. Im Labor stellen wir das später auf die `ROS_DOMAIN_ID` Ihrer Gruppe um.

## Selbsttest

Führen Sie alle drei Tests aus. Wenn alle klappen, sind Sie für Ü 2 bereit.

**Test 1: Umgebung**

```text
> printenv ROS_DISTRO
jazzy
> ros2 doctor
...
All 5 checks passed
```

**Test 2: Kommunikation zwischen C++ und Python** — in zwei Terminals:

```text
# Terminal 1
> ros2 run demo_nodes_cpp talker
[INFO] [...] [talker]: Publishing: 'Hello World: 1'
[INFO] [...] [talker]: Publishing: 'Hello World: 2'
```

```text
# Terminal 2
> ros2 run demo_nodes_py listener
[INFO] [...] [listener]: I heard: [Hello World: 1]
[INFO] [...] [listener]: I heard: [Hello World: 2]
```

Beenden Sie beide mit `Ctrl+C`.

**Test 3: Grafik**

```bash
ros2 run turtlesim turtlesim_node
```

Es öffnet sich ein Fenster mit einer Schildkröte auf blauem Grund.

**Checkliste:**

- [ ] Test 1: `ROS_DISTRO` ist `jazzy`, `ros2 doctor` meldet `All 5 checks passed` (einzelne Warnungen, etwa zum Netzwerk, sind unkritisch)
- [ ] Test 2: Der Listener empfängt die Nachrichten des Talkers
- [ ] Test 3: Das turtlesim-Fenster erscheint

## Wenn etwas nicht klappt

<!-- data-type="none" -->
| Problem                                           | Ursache und Abhilfe                                                                                     |
| :------------------------------------------------ | :------------------------------------------------------------------------------------------------------ |
| `ros2: command not found`                         | Umgebung nicht geladen: `source /opt/ros/jazzy/setup.bash` — und prüfen, ob die Zeile in `~/.bashrc` steht |
| `Unable to locate package ros-jazzy-desktop`      | Paketquelle fehlt oder falsche Ubuntu-Version: Schritt 1 wiederholen, `lsb_release -a` prüfen            |
| Fehler beim `apt install` wegen Abhängigkeiten    | vorher `sudo apt upgrade` ausführen; siehe Warnung zu `noble-updates` in der offiziellen Anleitung      |
| Listener empfängt nichts                          | beide Terminals mit derselben Umgebung? Die ROS-Variablen in beiden Terminals vergleichen (Befehl siehe unten)                     |
| `wsl --install` meldet Fehler `0x80370102`        | Virtualisierung ist im BIOS/UEFI abgeschaltet: im BIOS „Intel VT-x“ bzw. „AMD-V/SVM“ aktivieren          |
| Es öffnet sich ein Ubuntu mit anderer Version     | `lsb_release -a` prüfen; „Ubuntu 24.04“ gezielt im Startmenü bzw. Windows Terminal auswählen            |
| turtlesim-Fenster erscheint nicht (WSL2)          | WSL aktualisieren: `wsl --update` in der PowerShell, dann `wsl --shutdown` und Ubuntu neu öffnen         |

Die ROS-Variablen eines Terminals zeigt `printenv | grep ROS`.

Hilft das nicht, bringen Sie Ihren Laptop und die genaue Fehlermeldung zur Übung mit — oder arbeiten Sie dort zunächst am Pool-Rechner.
