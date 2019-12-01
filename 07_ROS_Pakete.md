<!--

author:   Sebastian Zug & Georg Jäger
email:    sebastian.zug@informatik.tu-freiberg.de & Georg.Jaeger@informatik.tu-freiberg.de
version:  1.0.0
language: de
comment: In dieser Veranstaltung der Vorlesung werden die Paket und Buildkonzepte von ROS2 vorgestellt. Dabei entsteht ein einfaches eigenes Package.
narrator: Deutsch Female

import: https://raw.githubusercontent.com/LiaTemplates/Rextester/master/README.md
-->

# Vorlesung VII - ROS2 Pakete

Eine interaktive Version des Kurses finden Sie unter [Link](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/SoftwareprojektRobotik/master/07_ROS_Pakete.md)

**Zielstellung der heutigen Veranstaltung**

+

--------------------------------------------------------------------------------


## Konzept

ROS1 und 2 sind in Pakten organisiert, diese kann man als Container für zusammengehörigen Code betrachten. Wenn Sie Ihren Code installieren oder mit anderen teilen möchten, muss dieser in einem Paket organisiert sein.

```bash    InspectPackages
> ros2 pkg
usage: ros2 pkg [-h] Call `ros2 pkg <command> -h` for more detailed usage. ...

Various package related sub-commands

optional arguments:
  -h, --help            show this help message and exit

Commands:
  create       Create a new ROS2 package
  executables  Output a list of package specific executables
  list         Output a list of available packages
  prefix       Output the prefix path of a package

  Call `ros2 pkg <command> -h` for more detailed usage.
zug@humboldt:~$ ros2 pkg list
action_msgs
action_tutorials
actionlib_msgs
ament_cmake
ament_cmake_auto
ament_cmake_copyright
```

Eine Paket umfasst aber nicht nur den eigentlichen Code sondern auch:

+ eine Spezifikation der Abhängigkeiten
+ die Konfiguration des Build-Systems
+ die Definition der Nutzerspezifischen Messages
+ die `launch` Files für den Start der Anwendungen und deren Parameterisierung

Ein minimales Paket umfasst zwei Dateien:

+ package.xml - Informationen über das Paket selbst (Autor, Version, )
+ CMakeLists.txt file - beschreibt wie das Paket gebaut wird und welche Abhängigkeiten bestehen

Die Paketerstellung in ROS 2 verwendet [ament](https://design.ros2.org/articles/ament.html) als Build-System und [colcon](https://colcon.readthedocs.io/en/released/user/quick-start.html) als Build-Tool.

Pakete können in gemeinsamen `Workspaces` angelegt werden.

```
workspace_folder/
    src/
      package_1/
          CMakeLists.txt
          package.xml

      package_2/
          setup.py
          package.xml
          resource/my_package
      ...
      package_n/
          CMakeLists.txt
          package.xml
```

Diese Struktur wird durch das jeweilige Build-System automatisch erweitert. `colcon` erstellt standardmäßig weitere Verzeichnisse in der Struktur des Projektes:

+ Das `build`-Verzeichnis befindet sich dort, wo Zwischendateien gespeichert werden. Für jedes Paket wird ein Unterordner erstellt, in dem z.B. CMake aufgerufen wird.

+ Das Installationsverzeichnis ist der Ort, an dem jedes Paket installiert wird. Standardmäßig wird jedes Paket in einem separaten Unterverzeichnis installiert.

+ Das `log` Verzeichnis enthält verschiedene Protokollinformationen zu jedem Colcon-Aufruf.

## Realisierung eines eigenen Paketes

Wir wollen die Funktionalität der `minimal_subscriber`/`minimal_publisher` Beispiel erweitern und einen neuen Knoten implementieren, der den Zählwert nicht als Bestandteil eines strings kommuniziert sondern als separaten Zahlenwert.

Sie finden den Beispielcode im Repository dieses Kurses unter [Link]

**Stufe 1: Individuelles Msg-Format**

```
> ros2 pkg create my_msg_package --build-type ament_cmake --dependencies rclcpp std_msgs
going to create a new package
package name: my_msg_package
destination directory: /home/zug/Desktop/SoftwareprojektRobotik/examples/07_ROS2Pakete/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['zug <Sebastian.Zug@informatik.tu-freiberg.de>']
licenses: ['TODO: License declaration']
build type: ament_cmake
dependencies: ['rclcpp', 'std_msgs']
creating folder ./my_msg_package
creating ./my_msg_package/package.xml
creating source and include folder
creating folder ./my_msg_package/src
creating folder ./my_msg_package/include/my_msg_package
creating ./my_msg_package/CMakeLists.txt
```

Ergänzungen ...

```bash           MyMsg.msg
int32 counter
string comment "Keine Evaluation des Wertes"
```

```
> colcon build --symlink-install
Starting >>> my_msg_package
Finished <<< my_msg_package [4.59s]                       

Summary: 1 package finished [4.68s]
```

Lassen Sie uns das neue Paket zunächst auf der Kommandozeile testen. Dazu wird
das neu definierte Work-Package in die Liste der verfügbaren Pakete aufgenommen.

```bash  ShellA
> source install/setup.bash
> ros2 msg list | grep my
my_msg_package/msg/MyMsg
ros2 topic pub /tralla my_msg_package/msg/MyMsg "{counter: '8'}"
```

```bash  ShellB
> source install/setup.bash
> ros2 topic echo /tralla
```

**Schritt 2: Integration einer Methode**



```
> ros2 pkg create --build-type ament_cmake --node-name data_generator my_tutorial_package
going to create a new package
package name: my_tutorial_package
destination directory: /home/zug/Desktop/SoftwareprojektRobotik/examples/07_ROS2Pakete/dev_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['zug <Sebastian.Zug@informatik.tu-freiberg.de>']
licenses: ['TODO: License declaration']
build type: ament_cmake
dependencies: []
node_name: data_generator
creating folder ./my_tutorial_package
creating ./my_tutorial_package/package.xml
creating source and include folder
creating folder ./my_tutorial_package/src
creating folder ./my_tutorial_package/include/my_tutorial_package
creating ./my_tutorial_package/CMakeLists.txt
creating ./my_tutorial_package/src/data_generator.cpp
```

```
colcon build --symlink-install
Starting >>> my_msg_package
Starting >>> my_tutorial_package
Finished <<< my_msg_package [0.59s]                                      
Finished <<< my_tutorial_package [1.75s]                       

Summary: 2 packages finished [1.86s]
```

Bevor Sie eine der installierten ausführbaren Dateien oder Bibliotheken verwenden können, müssen Sie sie zu Ihrem Pfad und Bibliothekspfaden hinzufügen. colcon hat bash/bat-Dateien im Installationsverzeichnis generiert, um die Einrichtung der Umgebung zu erleichtern.

```bash
> ros2 run my_tutorial_package data_generator
hello world my_tutorial_package package
```

Soweit so gut. Nun müssen wir allerdings auch noch die Logik in die Anwendung integrieren.


https://index.ros.org/doc/ros2/Tutorials/Rosidl-Tutorial/

## Aufgabe der Woche

+ Implementieren Sie einen Subscriber für das oben beschriebene Beispiel. 
