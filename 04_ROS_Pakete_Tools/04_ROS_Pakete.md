<!--

author:   Sebastian Zug & Georg Jäger & Claude.ai & Claude Code (Opus 5.5) mit LiaScript Teaching-Agent
email:    sebastian.zug@informatik.tu-freiberg.de & Georg.Jaeger@informatik.tu-freiberg.de
version:  0.2.0
language: de
narrator: Deutsch Female

import:   https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/config.md

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/04_ROS_Pakete_Tools/04_ROS_Pakete.md)

# ROS 2 Pakete

| Parameter            | Kursinformationen                                                                                                      |
| -------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| **Veranstaltung:**   | @config.lecture                                                                                                        |
| **Semester**         | @config.semester                                                                                                       |
| **Hochschule:**      | `Technische Universität Freiberg`                                                                                      |
| **Inhalte:**         | `Eigene ROS-2-Pakete anlegen und bauen, eigene Nachrichtentypen, Launch-Dateien, Logging, verteilte Systeme`           |
| **Link auf GitHub:** | https://github.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/blob/master/04_ROS_Pakete_Tools/04_ROS_Pakete.md     |
| **Autoren**          | @author                                                                                                                |

**Zum Einstieg:** Wenn Sie im Praktikum den TurtleBot hochfahren, tippen Sie genau einen Befehl:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

Dieser eine Befehl startet den LiDAR-Treiber, die Motorsteuerung und einen Knoten, der die Geometrie des Roboters veröffentlicht — verteilt auf drei Launch-Dateien in zwei Paketen. In VL 2 brauchten wir für zwei Knoten noch zwei Terminals. **Wie kommen wir von dort zu einem Befehl für ein ganzes System?** Das ist die Frage dieser Vorlesung.

--------------------------------------------------------------------------------

**Zielstellung der heutigen Veranstaltung**

In VL 2 haben wir fertige Knoten gestartet und untersucht. Heute schreiben wir eigene — so, wie Sie später Ihre Spurerkennung und Ihr Fahrverhalten für den TurtleBot umsetzen.

+ Aufbau eines ROS-2-Pakets und eines Workspace erklären
+ Pakete in C++ und Python anlegen, mit `colcon` bauen und starten
+ Einen eigenen Nachrichtentyp definieren und in beiden Sprachen verwenden
+ Mehrere Knoten mit einer Launch-Datei gemeinsam starten und konfigurieren
+ Log-Ausgaben gezielt einsetzen und ROS 2 über mehrere Rechner betreiben

--------------------------------------------------------------------------------

## Vom Beispiel zum eigenen Paket

    --{{0}}--
Ab heute schreiben Sie eigenen Code für ROS 2. Damit ROS ihn finden, bauen und starten kann, muss er in einem Paket liegen. Ein Paket ist nichts Magisches — ein Ordner mit Code und einer Beschreibung, die ROS lesen kann.

Bisher haben wir mit `ros2 run` Programme gestartet, die mit ROS 2 installiert wurden. Für das Projekt brauchen Sie eigene Knoten — eine Spurerkennung, eine Ampelerkennung, ein Fahrverhalten. Damit ROS diese finden, bauen und starten kann, müssen sie in **Paketen** organisiert sein.

Ein Paket ist ein Ordner mit zusammengehörigem Code und einer Beschreibung, die ROS lesen kann. Es enthält

+ den Quellcode der Knoten (C++ oder Python),
+ eine Liste der Abhängigkeiten, also anderer Pakete, die gebraucht werden,
+ die Anweisungen zum Bauen,
+ bei Bedarf eigene Nachrichtentypen und Launch-Dateien.

Welche Pakete installiert sind, verrät `ros2 pkg list`. Wie fein sollte man ein System in Pakete aufteilen? Einige Grundsätze aus dem Software-Engineering helfen:

+ Jedes Paket hat **eine** Aufgabe — die Spurerkennung weiß nichts vom Motor.
+ Pakete sind nur über Nachrichten gekoppelt, nicht über gemeinsamen Code.
+ Was in einem anderen Projekt wiederverwendbar sein soll (z.B. eigene Nachrichtentypen), kommt in ein eigenes Paket.

### Workspace und Paketstruktur

    --{{0}}--
Alle Ihre Pakete liegen in einem Arbeitsordner, dem Workspace. Merken Sie sich die vier Ordner: src schreiben Sie selbst, build, install und log erzeugt das Bauwerkzeug.

Pakete liegen in einem **Workspace**, einem Arbeitsordner mit dem Unterordner `src`. Angelegt ist er mit einem Befehl:

```bash
mkdir -p ~/ros2_ws/src      # -p legt auch den übergeordneten Ordner an (Ü 1)
```

Ein Workspace kann viele Pakete enthalten — C++ und Python gemischt. Am Ende dieser Vorlesung sieht unserer so aus:

```text
ros2_ws/
└── src/
    ├── my_msg_package/          ← C++/CMake-Paket (Nachrichtentyp)
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── msg/MyMsg.msg
    ├── my_tutorial_package/     ← C++-Paket (Knoten)
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── src/data_generator.cpp
    │   └── launch/counter.launch.py
    └── my_python_subscriber/    ← Python-Paket (Knoten)
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/my_python_subscriber
        └── my_python_subscriber/data_receiver.py
```

Zwei Dateien sind in jedem C++-Paket Pflicht:

+ `package.xml` — Name, Version, Autor, Lizenz und **Abhängigkeiten** des Pakets
+ `CMakeLists.txt` — wie das Paket gebaut wird (bei Python-Paketen übernimmt das `setup.py`)

Beim Bauen legt `colcon` neben `src` drei weitere Ordner an:

+ `build/` — Zwischendateien des Compilers
+ `install/` — die fertigen Programme und das Skript `setup.bash`
+ `log/` — Protokolle jedes Build-Vorgangs

> Diese drei Ordner gehören nicht in Git. Sie lassen sich jederzeit neu erzeugen.

## Bauen mit colcon

    --{{0}}--
Warum brauchen wir ein eigenes Bauwerkzeug? Weil Pakete voneinander abhängen. colcon findet die richtige Reihenfolge und legt alles so ab, dass ROS es findet. Der häufigste Fehler danach ist das vergessene source.

Warum reicht nicht einfach `g++ main.cpp`? Bei einem einzelnen Programm würde das genügen. In einem Workspace hängen Pakete aber voneinander ab: Der Knoten in `my_tutorial_package` braucht den Nachrichtentyp aus `my_msg_package`, der vorher in C++- und Python-Code übersetzt werden muss. Ein Build-Werkzeug muss also

+ die Abhängigkeiten aller Pakete erkennen und eine Reihenfolge festlegen,
+ jedes Paket mit seinem eigenen Build-System bauen (CMake für C++, setuptools für Python),
+ die Ergebnisse so installieren, dass ROS sie findet.

ROS 2 teilt diese Aufgabe auf zwei Werkzeuge auf:

<!-- data-type="none" -->
| Werkzeug   | Ebene          | Aufgabe                                                                            |
| :--------- | :------------- | :--------------------------------------------------------------------------------- |
| **ament**  | ein Paket      | Build-Regeln für ein einzelnes Paket: `ament_cmake` (C++) oder `ament_python`      |
| **colcon** | ganzer Workspace | baut alle Pakete in der richtigen Reihenfolge und erzeugt `install/setup.bash`  |

Gebaut wird immer im Wurzelordner des Workspace, nicht in `src`:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Danach muss die Shell den neuen Inhalt kennen. Wie in Ü 1 bei `source /opt/ros/jazzy/setup.bash` erweitert `install/setup.bash` die Umgebungsvariablen — diesmal um Ihre eigenen Pakete:

```bash
source ~/ros2_ws/install/setup.bash
```

> **Merke:** Erst ROS laden, dann bauen, dann den Workspace laden — und das in **jedem neuen Terminal** erneut. Ein neues Terminal startet in Ihrem Home-Verzeichnis; der relative Pfad `source install/setup.bash` funktioniert nur, wenn Sie vorher mit `cd ~/ros2_ws` in den Workspace gewechselt sind. Mit dem vollständigen Pfad `~/ros2_ws/install/setup.bash` sind Sie auf der sicheren Seite.

Zwei nützliche Optionen:

+ `--symlink-install` legt in `install/` nur Verweise (_Links_) auf Ihre Python- und Launch-Dateien an, statt sie zu kopieren. Änderungen am Code wirken dann sofort, ohne neuen Build. Ändern Sie dagegen `setup.py`, `package.xml` oder C++-Code, müssen Sie neu bauen.
+ `--packages-select <paket>` baut nur das genannte Paket — spart Zeit, wenn Sie nur an einem Paket arbeiten.

**Kurze Kontrolle:** Sie haben `colcon build` erfolgreich ausgeführt, aber `ros2 run my_tutorial_package data_generator` meldet `Package 'my_tutorial_package' not found`. Was fehlt?

[( )] Das Paket muss zusätzlich mit `sudo apt install` installiert werden.
[(X)] In diesem Terminal wurde `source install/setup.bash` nicht ausgeführt.
[( )] `colcon build` muss im Ordner `src` aufgerufen werden.
***
`colcon build` legt die Programme in `install/` ab. Welche Pakete `ros2` kennt, steht aber in den Umgebungsvariablen der Shell — erst `source install/setup.bash` trägt den Workspace dort ein. Das gilt für jedes Terminal einzeln (vgl. Ü 1).
***

## Ein eigenes Paket Schritt für Schritt

    --{{0}}--
Wir bauen jetzt in drei Schritten ein kleines System: einen eigenen Nachrichtentyp, einen Publisher in C++ und einen Subscriber in Python. Achten Sie darauf, was in jedem Schritt gleich bleibt — das ist das Muster für Ihre eigenen Pakete.

Wir erweitern das Publisher/Subscriber-Beispiel aus VL 2. Statt eines Textes soll der Zählwert als Zahl übertragen werden, zusammen mit einem Kommentar. Dazu brauchen wir

1. einen eigenen Nachrichtentyp (Paket `my_msg_package`),
2. einen Publisher in C++ (Paket `my_tutorial_package`),
3. einen Subscriber in Python (Paket `my_python_subscriber`).

Den vollständigen, unter Jazzy getesteten Code finden Sie im Repository unter [04_ROS_Pakete_Tools/example](https://github.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/tree/master/04_ROS_Pakete_Tools/example/pkg_example).

### Schritt 1: Eigener Nachrichtentyp

    --{{0}}--
Für die Verkehrswelt brauchen Sie später eigene Nachrichten, etwa für den Zustand einer Ampel. Wie das geht, zeige ich an einem einfachen Zähler. Wichtig: Wir testen den neuen Typ zuerst nur mit der Kommandozeile, noch ohne eigenen Code.

Neue Pakete legt `ros2 pkg create` an. Wir starten im Ordner `src` des Workspace:

```text
> cd ~/ros2_ws/src
> ros2 pkg create my_msg_package --build-type ament_cmake --license MIT
going to create a new package
package name: my_msg_package
...
build type: ament_cmake
dependencies: []
creating folder ./my_msg_package
creating ./my_msg_package/package.xml
creating source and include folder
creating folder ./my_msg_package/src
creating folder ./my_msg_package/include/my_msg_package
creating ./my_msg_package/CMakeLists.txt
```

Im Paket legen wir einen Ordner `msg` an (`mkdir ~/ros2_ws/src/my_msg_package/msg`) und darin die Datei `MyMsg.msg`. Sie beschreibt die Felder der Nachricht — ein Feld pro Zeile, erst der Typ, dann der Name, optional ein Standardwert:

```text  msg/MyMsg.msg
int32 counter
string comment "Keine Evaluation des Wertes"
```

Aus dieser Beschreibung muss beim Bauen C++- und Python-Code entstehen. Das erledigt ein Code-Generator von ROS, das Paket `rosidl_default_generators`. Wir tragen ihn in die `CMakeLists.txt` ein — direkt nach der Zeile `find_package(ament_cmake REQUIRED)` und in jedem Fall vor der letzten Zeile `ament_package()`:

```cmake  CMakeLists.txt
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyMsg.msg"
)
```

und in die `package.xml`, innerhalb von `<package>` und vor dem Abschnitt `<export>`:

```xml  package.xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Die `package.xml` unterscheidet, **wofür** eine Abhängigkeit gebraucht wird: `buildtool_depend` nur zum Bauen (der Generator), `exec_depend` nur zur Laufzeit, `depend` für beides — das ist der Normalfall. `member_of_group` meldet das Paket als Nachrichtenpaket an.

Bauen und testen — noch ganz ohne eigenen Knoten, nur mit der Kommandozeile:

```text
> cd ~/ros2_ws
> colcon build --symlink-install
Starting >>> my_msg_package
Finished <<< my_msg_package [5.07s]

Summary: 1 package finished [5.22s]
> source install/setup.bash
> ros2 interface show my_msg_package/msg/MyMsg
int32 counter
string comment "Keine Evaluation des Wertes"
> ros2 topic pub --rate 2 /counter my_msg_package/msg/MyMsg "{counter: 8}"
```

```text
# zweites Terminal
> source /opt/ros/jazzy/setup.bash
> source ~/ros2_ws/install/setup.bash
> ros2 topic echo --once /counter
counter: 8
comment: Keine Evaluation des Wertes
---
```

Das Feld `comment` haben wir beim Senden nicht angegeben — es enthält trotzdem den Standardwert aus der `.msg`-Datei.

> **Stolperfalle:** Starten Sie `ros2 topic echo` für einen eigenen Nachrichtentyp, bevor ein Publisher läuft, meldet ROS `Could not determine the type for the passed topic`. Starten Sie dann zuerst den Publisher oder geben Sie den Typ mit an: `ros2 topic echo /counter my_msg_package/msg/MyMsg`.

Eine ausführliche Anleitung, auch zu eigenen Service-Typen, bietet das Tutorial [Creating custom msg and srv files](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html).

### Schritt 2: Publisher in C++

    --{{0}}--
Der Publisher ist fast der aus VL 2. Neu sind nur zwei Dinge: der eigene Nachrichtentyp und die Einträge in package.xml und CMakeLists.txt, mit denen das Paket seine Abhängigkeit erklärt.

Das Paket für den Knoten legen wir gleich mit seinen Abhängigkeiten und einem Programmgerüst an:

```text
> cd ~/ros2_ws/src
> ros2 pkg create my_tutorial_package --build-type ament_cmake --license MIT --node-name data_generator --dependencies rclcpp my_msg_package
...
dependencies: ['rclcpp', 'my_msg_package']
node_name: data_generator
...
creating ./my_tutorial_package/src/data_generator.cpp
```

Durch `--dependencies` stehen die Abhängigkeiten bereits in beiden Beschreibungsdateien:

```xml  package.xml
<depend>rclcpp</depend>
<depend>my_msg_package</depend>
```

```cmake  CMakeLists.txt
find_package(rclcpp REQUIRED)
find_package(my_msg_package REQUIRED)

add_executable(data_generator src/data_generator.cpp)
ament_target_dependencies(data_generator "rclcpp" "my_msg_package")

install(TARGETS data_generator
  DESTINATION lib/${PROJECT_NAME})
```

CMake ist das Build-System für C++, das hinter `ament_cmake` steckt. Die Befehle im Überblick:

<!-- data-type="none" -->
| CMake-Befehl                  | Bedeutung                                                                     |
| :---------------------------- | :---------------------------------------------------------------------------- |
| `find_package(xyz REQUIRED)`  | Paket `xyz` suchen; Abbruch mit Fehlermeldung, wenn es nicht installiert ist  |
| `add_executable(name datei)`  | aus der Quelldatei ein Programm `name` kompilieren                            |
| `ament_target_dependencies`   | dem Programm die Header und Bibliotheken der genannten Pakete bereitstellen   |
| `install(TARGETS ...)`        | das fertige Programm nach `install/` kopieren, damit `ros2 run` es findet      |

Das Gerüst `data_generator.cpp` ersetzen wir durch den Publisher aus VL 2 — mit zwei Änderungen: dem neuen Nachrichtentyp und dem Topic `counter`.

```cpp  data_generator.cpp
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "my_msg_package/msg/my_msg.hpp"      // aus MyMsg.msg generiert

using namespace std::chrono_literals;

class DataGenerator : public rclcpp::Node
{
public:
  DataGenerator()
  : Node("data_generator"), count_(0)
  {
    publisher_ = this->create_publisher<my_msg_package::msg::MyMsg>("counter", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&DataGenerator::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = my_msg_package::msg::MyMsg();
    message.counter = count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: %d, %s",
      message.counter, message.comment.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<my_msg_package::msg::MyMsg>::SharedPtr publisher_;
  int32_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataGenerator>());
  rclcpp::shutdown();
  return 0;
}
```

Beachten Sie den Zusammenhang der Namen: Aus der Datei `MyMsg.msg` im Paket `my_msg_package` wird der Header `my_msg_package/msg/my_msg.hpp` und der C++-Typ `my_msg_package::msg::MyMsg`. Die Felder `counter` und `comment` sind ganz normale Membervariablen.

**Vorhersage, bevor wir weitermachen:** Der Subscriber soll in Python geschrieben werden. Was erwarten Sie — welche Datei übernimmt dort die Rolle von `add_executable` in der `CMakeLists.txt`?

[( )] ebenfalls eine `CMakeLists.txt`
[(X)] eine Python-Datei, die das Paket beschreibt (`setup.py`)
[( )] keine — Python-Dateien sind direkt ausführbar, ROS findet sie selbst
***
Auch ein Python-Programm muss ROS bekannt gemacht werden, sonst findet `ros2 run` es nicht. Das übernimmt der Eintrag `entry_points` in `setup.py` — gleich im nächsten Schritt.
***

### Schritt 3: Subscriber in Python

    --{{0}}--
Jetzt die Gegenseite in Python. Die Struktur des Knotens kennen Sie schon. Neu ist, wie ein Python-Programm angemeldet wird. Am Ende sprechen ein C++-Knoten und ein Python-Knoten miteinander.

Python-Pakete werden mit `--build-type ament_python` angelegt:

```text
> ros2 pkg create my_python_subscriber --build-type ament_python --license MIT --dependencies rclpy my_msg_package
...
creating ./my_python_subscriber/setup.py
creating ./my_python_subscriber/setup.cfg
creating folder ./my_python_subscriber/resource
creating ./my_python_subscriber/resource/my_python_subscriber
creating ./my_python_subscriber/my_python_subscriber/__init__.py
...
```

Den Knoten legen wir in den inneren Ordner, der wie das Paket heißt: `my_python_subscriber/my_python_subscriber/data_receiver.py`.

```python  data_receiver.py
import rclpy
from rclpy.node import Node

from my_msg_package.msg import MyMsg


class DataReceiver(Node):

    def __init__(self):
        super().__init__('data_receiver')
        self.subscription = self.create_subscription(
            MyMsg, 'counter', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.counter} ({msg.comment})')


def main(args=None):
    rclpy.init(args=args)
    node = DataReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

Der Aufbau entspricht dem C++-Knoten aus VL 2: eine Klasse, die von `Node` erbt, im Konstruktor (`__init__`) wird mit `create_subscription(Typ, Topic, Callback, Puffergröße)` das Abonnement angelegt, und `rclpy.spin()` wartet auf Nachrichten. `try/finally` sorgt dafür, dass der Knoten auch bei `Ctrl+C` sauber beendet wird.

Anders als in C++ gibt es kein `add_executable`. Stattdessen meldet ein Eintrag in `setup.py` die Funktion `main` als ausführbares Programm `data_receiver` an — das ist der Name, den `ros2 run` erwartet. Die generierte `setup.py` enthält bereits eine **leere** Liste `'console_scripts': []`; ergänzen Sie dort die Zeile, statt einen zweiten Block einzufügen:

```python  setup.py
    entry_points={
        'console_scripts': [
            'data_receiver = my_python_subscriber.data_receiver:main',
        ],
    },
```

Die Abhängigkeit zu `my_msg_package` steht durch `--dependencies` bereits in der `package.xml`. Jetzt bauen wir alle drei Pakete gemeinsam — `colcon` erkennt selbst, dass `my_msg_package` zuerst an der Reihe ist:

```text
> cd ~/ros2_ws
> colcon build --symlink-install
Starting >>> my_msg_package
Finished <<< my_msg_package [0.46s]
Starting >>> my_python_subscriber
Starting >>> my_tutorial_package
Finished <<< my_python_subscriber [1.95s]
Finished <<< my_tutorial_package [4.81s]

Summary: 3 packages finished [5.42s]
```

Die Abhängigkeiten zwischen den Paketen gibt `colcon graph --dot` aus:

```text
> colcon graph --dot
digraph graphname {
  "my_tutorial_package";
  "my_python_subscriber";
  "my_msg_package";
  "my_tutorial_package" -> "my_msg_package" [color="#0000ff:#ff0000"];
  "my_python_subscriber" -> "my_msg_package" [color="#0000ff:#ff0000"];
}
```

Mit dem Werkzeug Graphviz wird daraus ein Bild (`colcon graph --dot | dot -Tpng -o graph.png`):

![Abhängigkeitsgraph: my_tutorial_package und my_python_subscriber hängen beide von my_msg_package ab](images/colcon_graph.png)<!-- width="55%" -->

Test in zwei Terminals (jeweils nach `source /opt/ros/jazzy/setup.bash` und `source ~/ros2_ws/install/setup.bash`):

```text
# Terminal 1
> ros2 run my_tutorial_package data_generator
[INFO] [...] [data_generator]: Publishing: 0, Keine Evaluation des Wertes
[INFO] [...] [data_generator]: Publishing: 1, Keine Evaluation des Wertes
```

```text
# Terminal 2
> ros2 run my_python_subscriber data_receiver
[INFO] [...] [data_receiver]: Received: 1 (Keine Evaluation des Wertes)
[INFO] [...] [data_receiver]: Received: 2 (Keine Evaluation des Wertes)
```

Ein C++-Knoten und ein Python-Knoten tauschen also problemlos einen selbst definierten Nachrichtentyp aus. Warum empfängt der Subscriber erst ab 1? Er wurde nach dem Publisher gestartet — Nachrichten, die vor seinem Start gesendet wurden, verpasst er. Topics sind ein Live-Strom, keine Ablage.

**C++- und Python-Pakete im Vergleich**

<!-- data-type="none" -->
| Aspekt                  | C++ (`ament_cmake`)                     | Python (`ament_python`)                      |
| :---------------------- | :-------------------------------------- | :------------------------------------------- |
| Build-Beschreibung      | `CMakeLists.txt`                        | `setup.py`, `setup.cfg`                      |
| Quellcode-Ordner        | `src/`, `include/`                      | `<paketname>/`                               |
| Programm anmelden       | `add_executable()` + `install()`        | `entry_points` in `setup.py`                 |
| Abhängigkeiten          | `package.xml` (`<depend>`) und `find_package()` | `package.xml` (`<depend>`)           |
| Nachrichtentyp einbinden | `#include "pkg/msg/my_msg.hpp"`        | `from pkg.msg import MyMsg`                  |
| Nach Codeänderung       | neu bauen                               | mit `--symlink-install` sofort wirksam; nach Änderung an `setup.py` neu bauen |

> Eigene Nachrichtentypen definieren Sie immer in einem CMake-Paket — auch wenn alle Knoten in Python geschrieben sind.

**Kurze Kontrolle:** Sie legen im Paket `my_python_subscriber` eine neue Datei `lane_detector.py` mit einem Knoten und einer Funktion `main` an und bauen den Workspace. `ros2 run my_python_subscriber lane_detector` meldet `No executable found`. Was fehlt am wahrscheinlichsten?

[( )] Ein Eintrag `add_executable(lane_detector ...)` in der `CMakeLists.txt`.
[(X)] Ein Eintrag für `lane_detector` unter `console_scripts` in `setup.py` — und danach ein neuer Build.
[( )] Die Abhängigkeit `rclcpp` in der `package.xml`.
***
Python-Pakete haben keine `CMakeLists.txt`; ausführbare Programme werden über `entry_points` in `setup.py` angemeldet. Weil sich damit die Paketbeschreibung ändert, ist ein erneutes `colcon build` nötig — `--symlink-install` hilft nur bei Änderungen am Code selbst.
***

## Systeme starten mit Launch-Dateien

    --{{0}}--
Zurück zur Einstiegsfrage: Für zwei Knoten brauchen wir schon zwei Terminals. Eine Launch-Datei beschreibt, welche Knoten mit welchen Einstellungen starten. Ein Befehl genügt dann für das ganze System.

Sie haben gesehen: Für zwei Knoten brauchen wir zwei Terminals, jeweils mit zwei `source`-Befehlen. Unser TurtleBot-System wird aus Kameratreiber, Spurerkennung, Ampelerkennung und Fahrverhalten bestehen — dazu Parameter und Umbenennungen. Das von Hand zu starten ist fehleranfällig.

Eine **Launch-Datei** beschreibt, welche Knoten mit welchen Einstellungen gestartet werden. ROS 2 akzeptiert Launch-Dateien in Python, XML oder YAML; wir verwenden Python. Unsere Datei startet Publisher und Subscriber gemeinsam und erlaubt, den Topic-Namen beim Start festzulegen:

```python  launch/counter.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic = LaunchConfiguration('topic')
    return LaunchDescription([
        DeclareLaunchArgument('topic', default_value='counter',
                              description='Name des Topics für die Zählwerte'),
        Node(package='my_tutorial_package', executable='data_generator',
             remappings=[('counter', topic)], output='screen'),
        Node(package='my_python_subscriber', executable='data_receiver',
             remappings=[('counter', topic)], output='screen'),
    ])
```

| Element                   | Bedeutung                                                                                     |
| :------------------------ | :-------------------------------------------------------------------------------------------- |
| `generate_launch_description()` | Diese Funktion ruft `ros2 launch` auf; sie gibt die Liste der zu startenden Elemente zurück. |
| `DeclareLaunchArgument`   | ein Argument, das beim Start überschrieben werden kann, hier mit Standardwert `counter`       |
| `Node(...)`               | ein Knoten: Paket, Programm und Einstellungen                                                  |
| `LaunchConfiguration('topic')` | Platzhalter für den Wert des Arguments `topic` — wird erst beim Start eingesetzt          |
| `remappings`              | Umbenennung wie `--ros-args -r topic:=topic2` in VL 2                                          |
| `parameters=['datei.yaml']` | Parameter aus einer YAML-Datei laden — hier nicht verwendet, mehr zu Parametern in Ü 2      |
| `output='screen'`         | Logausgaben des Knotens im Terminal anzeigen                                                   |

Die Datei liegt im Ordner `launch` des Pakets `my_tutorial_package` (`mkdir ~/ros2_ws/src/my_tutorial_package/launch`). Eine Launch-Datei darf Knoten aus beliebigen Paketen starten — sie müssen nur gebaut und per `source` bekannt sein. In größeren Projekten legt man dafür oft ein eigenes Paket an, z.B. `verkehrswelt_bringup`.

Damit die Launch-Datei beim Bauen mit installiert wird, ergänzen wir in der `CMakeLists.txt` von `my_tutorial_package` vor `ament_package()`:

```cmake  CMakeLists.txt
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
```

Nach `colcon build` startet **ein** Befehl das ganze System:

```text
> ros2 launch my_tutorial_package counter.launch.py
[INFO] [launch]: All log files can be found below /home/.../.ros/log/...
[INFO] [data_generator-1]: process started with pid [202935]
[INFO] [data_receiver-2]: process started with pid [202936]
[data_generator-1] [INFO] [...] [data_generator]: Publishing: 0, Keine Evaluation des Wertes
[data_receiver-2] [INFO] [...] [data_receiver]: Received: 0 (Keine Evaluation des Wertes)
...
> ros2 launch my_tutorial_package counter.launch.py topic:=zaehler
```

Mit `topic:=zaehler` senden beide Knoten über `/zaehler` — prüfen Sie es mit `ros2 topic list`. Welche Argumente eine Launch-Datei kennt, zeigt `ros2 launch my_tutorial_package counter.launch.py --show-args`.

**Kurze Kontrolle:** Sie entfernen in der Launch-Datei die Zeile `remappings=[('counter', topic)]` beim **Subscriber** und starten mit `topic:=zaehler`. Was passiert?

[( )] Beide Knoten verwenden weiterhin `/zaehler`.
[(X)] Der Publisher sendet auf `/zaehler`, der Subscriber wartet auf `/counter` — er empfängt nichts.
[( )] `ros2 launch` bricht mit einer Fehlermeldung ab.
***
Das Remapping gilt nur für den Knoten, bei dem es eingetragen ist. Der Subscriber benutzt den Namen aus seinem Code, `counter`. Beide Knoten laufen fehlerfrei, sind aber nicht verbunden — ein typischer Fehler, den `ros2 topic info` oder `rqt_graph` sofort sichtbar machen.
***

Launch-Dateien können noch mehr: andere Launch-Dateien einbinden (`IncludeLaunchDescription`), Parameter aus YAML-Dateien laden, Knoten zeitverzögert starten (`TimerAction`) oder abhängig von Bedingungen. Beispiele zeigt das Tutorial [Launch](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html). Der TurtleBot selbst wird über die Launch-Datei `robot.launch.py` des Pakets `turtlebot3_bringup` gestartet — schauen Sie im ersten Praktikum hinein.

## Logging

    --{{0}}--
Statt print nutzen Sie den Logger. Er sagt Ihnen, wann und von welchem Knoten eine Meldung kommt, und Sie können die Menge der Meldungen beim Start einstellen, ohne den Code zu ändern.

Statt `printf` oder `print` nutzen ROS-Knoten den eingebauten Logger, der den Namen des Knotens trägt: `RCLCPP_INFO(...)` in C++, `self.get_logger().info(...)` in Python. Das hat Vorteile: Jede Meldung trägt Zeitstempel und Knotennamen, sie landet zusätzlich in Logdateien und auf dem Topic `/rosout`, und sie hat einen **Schweregrad**:

`DEBUG` < `INFO` < `WARN` < `ERROR` < `FATAL`

Beim Start lässt sich festlegen, ab welchem Schweregrad Meldungen erscheinen — ohne den Code zu ändern:

```bash
ros2 run my_tutorial_package data_generator --ros-args --log-level warn                  # nur noch Warnungen und Fehler
ros2 run my_tutorial_package data_generator --ros-args --log-level data_generator:=debug # Debug-Meldungen nur dieses Knotens
```

> **Stolperfalle:** `--log-level debug` ohne Knotennamen schaltet auch die internen Debug-Meldungen von ROS selbst frei — das Terminal wird mit Hunderten Zeilen geflutet. Geben Sie deshalb den Namen des Loggers an, hier `data_generator`.

<details>
<summary>Vertiefung: Logmeldungen filtern (zum Nachlesen)</summary>

Für C++ stellt `rclcpp` Varianten der Makros bereit, die Ausgaben filtern ([Doku: Logging](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Logging.html)). `x` steht für den Schweregrad, z.B. `RCLCPP_WARN_ONCE`:

<!-- data-type="none" -->
| Makro                  | Zusätzliche Argumente      | Ausgabe erfolgt …                                        |
| :--------------------- | :------------------------- | :------------------------------------------------------- |
| `RCLCPP_x`             | —                          | bei jedem Aufruf                                         |
| `RCLCPP_x_ONCE`        | —                          | nur beim ersten Aufruf                                   |
| `RCLCPP_x_SKIPFIRST`   | —                          | bei jedem Aufruf außer dem ersten                        |
| `RCLCPP_x_EXPRESSION`  | Bedingung                  | nur, wenn die Bedingung wahr ist                         |
| `RCLCPP_x_THROTTLE`    | Uhr, Zeitspanne in ms      | höchstens einmal pro Zeitspanne                          |

```cpp  Logging.cpp
// erscheint nur beim ersten Durchlauf
RCLCPP_INFO_ONCE(this->get_logger(), "Timer läuft");
// nur gerade Zählwerte, und nur bei --log-level debug
RCLCPP_DEBUG_EXPRESSION(this->get_logger(), (count_ % 2) == 0, "Zählwert %d ist gerade", count_);
// höchstens alle 2 Sekunden; *this->get_clock() übergibt die Uhr des Knotens
RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Kein Bild empfangen");
```

</details>

Gerade die Variante `RCLCPP_x_THROTTLE` (höchstens einmal pro Zeitspanne) ist im Projekt wichtig: Ein Knoten, der 30-mal pro Sekunde „Keine Spur erkannt“ meldet, macht das Terminal unlesbar.

> **Aufgabe:** Ändern Sie den `data_generator` so, dass die Logmeldung nur noch alle 10 Sekunden erscheint, die Nachrichten aber weiter zweimal pro Sekunde gesendet werden.

<details>
<summary>Lösung anzeigen</summary>

```cpp
RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
  "Publishing: %d, %s", message.counter, message.comment.c_str());
```

Nach der Änderung muss das C++-Paket neu gebaut werden (`colcon build --packages-select my_tutorial_package`).

</details>

## ROS 2 auf mehreren Rechnern

    --{{0}}--
Im Praktikum läuft Ihr System verteilt: die Treiber auf dem Roboter, Ihre Knoten auf dem Laptop. Programmieren müssen Sie dafür nichts. Entscheidend ist eine einzige Umgebungsvariable.

Im Praktikum läuft Ihr System verteilt: Auf dem Raspberry Pi des TurtleBot laufen die Treiber für LiDAR, Kamera und Motoren, auf Ihrem Laptop Ihre eigenen Knoten und die Visualisierung. Programmieren müssen Sie dafür nichts — dank Discovery (VL 2) finden sich die Knoten im Netz von selbst, wenn beide Rechner im selben Netz sind und **dieselbe `ROS_DOMAIN_ID`** verwenden.

Jede Gruppe bekommt eine eigene ID (0 bis 101), damit Ihr Laptop nicht die Fahrbefehle für den Roboter der Nachbargruppe sendet:

```bash
export ROS_DOMAIN_ID=12      # auf dem Laptop UND auf dem TurtleBot
ros2 topic list              # zeigt jetzt /scan, /odom, /cmd_vel ... des eigenen Roboters
```

Zu Hause wollen Sie dagegen niemanden im WLAN stören: Mit `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` sucht ROS 2 nur auf dem eigenen Rechner nach Knoten.

> **WSL2:** WSL2 hat standardmäßig ein eigenes, internes Netz, aus dem heraus der TurtleBot nicht sichtbar ist. Die Umstellung auf den gespiegelten Netzwerkmodus erledigen Sie vor dem ersten Praktikum zu Hause — die Anleitung steht in P 1 unter „Vorbereitung“. Für alle Übungen auf dem eigenen Rechner ist nichts zu tun.

## Zusammenfassung und Ausblick

    --{{0}}--
Halten wir den Weg vom Code zum laufenden System fest. Er ist für jedes Paket gleich und wird Ihnen in jedem Praktikum begegnen.

**Vom Code zum laufenden System**

<!-- data-type="none" -->
| Schritt              | Befehl / Datei                                                        |
| :------------------- | :-------------------------------------------------------------------- |
| 1. Paket anlegen     | `ros2 pkg create <name> --build-type ament_cmake` bzw. `ament_python` |
| 2. Beschreiben       | `package.xml` (Abhängigkeiten), `CMakeLists.txt` bzw. `setup.py` (Programme) |
| 3. Bauen             | `cd ~/ros2_ws && colcon build --symlink-install`                      |
| 4. Bekannt machen    | `source ~/ros2_ws/install/setup.bash` — in jedem neuen Terminal       |
| 5. Starten           | `ros2 run <paket> <programm>` oder `ros2 launch <paket> <datei>`      |

**Abschluss-Check**

Ordnen Sie die Dateien ihrem Zweck zu.

[ [package.xml] [CMakeLists.txt] [setup.py] [Launch-Datei] ]
[     (X)            ( )           ( )           ( )      ] Das Paket braucht `my_msg_package`.
[     ( )            (X)           ( )           ( )      ] Aus `data_generator.cpp` soll ein Programm entstehen.
[     ( )            ( )           (X)           ( )      ] Die Python-Funktion `main` soll als `data_receiver` startbar sein.
[     ( )            ( )           ( )           (X)      ] Publisher und Subscriber sollen gemeinsam mit einem Topic-Namen starten.
***
`package.xml` beschreibt, **was** ein Paket braucht; `CMakeLists.txt` und `setup.py` beschreiben, **wie** es gebaut wird und welche Programme entstehen; die Launch-Datei beschreibt, **wie das System gestartet** wird.
***

Ihr C++-Knoten gibt plötzlich nur noch Warnungen aus, Sie wollen aber die Debug-Meldungen **dieses einen** Knotens sehen. Welche Option hängen Sie an `ros2 run` an?

[( )] `--ros-args --log-level debug`
[(X)] `--ros-args --log-level data_generator:=debug`
[( )] `--log debug`
***
Ohne Knotennamen schaltet `--log-level debug` auch die internen Meldungen von ROS frei — das Terminal wird geflutet.
***

**Ausblick auf Block 5:** Große Systeme wie der Navigations-Stack Nav2 starten ihre Knoten nicht einfach gleichzeitig, sondern geordnet über einen _Lebenszyklus_ (konfigurieren, aktivieren, deaktivieren). Das lernen Sie in der Vorlesung zur Navigation kennen.

**Zurück zur Einstiegsfrage:** Der TurtleBot startet mit einem Befehl, weil seine Knoten in Paketen liegen, mit `colcon` gebaut sind und eine Launch-Datei sie gemeinsam startet. Genau so werden Sie Ihr Verkehrswelt-System aufbauen.

**Ausblick:** In **Ü 2** probieren Sie Services, Parameter und Actions am turtlesim aus und zeichnen Daten mit `ros2 bag` auf. Bringen Sie Ihre Skizze aus der Aufgabe der Woche (Nr. 3) mit — wir vergleichen sie zu Beginn.

## Aufgabe der Woche

    --{{0}}--
Bauen Sie die drei Pakete selbst nach — erst dann merken Sie, wo es hakt. Und bringen Sie Ihre Skizze für die Verkehrswelt zur Übung mit.

1. Legen Sie in Ihrer ROS-2-Umgebung (siehe [Installationsanleitung](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/exercises/installation_ros2/README.md)) einen Workspace `~/ros2_ws` an und bauen Sie die drei Pakete dieser Vorlesung nach. Prüfen Sie jeden Schritt mit `ros2 interface show`, `ros2 pkg executables` und `ros2 topic echo`.
   **Erwartetes Ergebnis:** `ros2 pkg executables my_tutorial_package` zeigt `my_tutorial_package data_generator`, und `ros2 launch my_tutorial_package counter.launch.py` liefert abwechselnd `Publishing:` und `Received:`.
2. Ergänzen Sie `MyMsg.msg` um ein Feld `float32 temperature` und senden Sie einen Zufallswert. Was müssen Sie neu bauen?
3. Erweitern Sie Ihre Graph-Skizze aus VL 2 (Aufgabe 3): Welche Pakete würden Sie für Spurerkennung, Ampelerkennung und Fahrverhalten anlegen? Wohin gehört ein eigener Nachrichtentyp „Ampelzustand“? Ordnen Sie Knoten und Topics den Paketen zu. Wir vergleichen die Skizzen zu Beginn von Ü 2.

<details>
<summary>Lösungshinweise zu Aufgabe 2 und 3 anzeigen</summary>

**Aufgabe 2:** `my_msg_package` muss neu gebaut werden, damit der Code für das neue Feld entsteht, und `my_tutorial_package`, weil der C++-Knoten gegen den geänderten Header kompiliert wird. Am einfachsten bauen Sie mit `colcon build` alle Pakete; `colcon` hält die Reihenfolge ein. Danach zeigt `ros2 topic echo /counter` das Feld `temperature`.

**Aufgabe 3 — eine mögliche Lösung:**

```text
verkehrswelt_msgs/        TrafficLight.msg   (eigener Nachrichtentyp, CMake-Paket)
lane_detection/           Knoten lane_detector     /camera/image_raw → /lane
traffic_light_detection/  Knoten light_detector    /camera/image_raw → /traffic_light (TrafficLight)
driving/                  Knoten driver            /lane, /traffic_light, /scan → /cmd_vel
verkehrswelt_bringup/     Launch-Datei, die alles zusammen mit dem TurtleBot startet
```

Der Nachrichtentyp liegt in einem eigenen Paket, damit Ampelerkennung und Fahrverhalten ihn beide verwenden können, ohne voneinander abzuhängen.

</details>

**Zum Weiterlesen**

+ [ROS 2 Jazzy: Creating a workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) und [Creating a package](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
+ [colcon Dokumentation](https://colcon.readthedocs.io/en/released/user/quick-start.html)
