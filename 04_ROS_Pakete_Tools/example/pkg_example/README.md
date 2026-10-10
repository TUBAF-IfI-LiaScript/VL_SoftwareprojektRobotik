# Beispielpakete zu VL 3 „ROS 2 Pakete“

_Autoren: Sebastian Zug & Claude Code (Opus 5.5) mit LiaScript Teaching-Agent — getestet mit ROS 2 Jazzy (Ubuntu 24.04), Oktober 2026_

| Paket                  | Typ            | Inhalt                                                      |
| :--------------------- | :------------- | :---------------------------------------------------------- |
| `my_msg_package`       | `ament_cmake`  | eigener Nachrichtentyp `MyMsg` (`int32 counter`, `string comment`) |
| `my_tutorial_package`  | `ament_cmake`  | C++-Publisher `data_generator`, Launch-Datei `counter.launch.py` |
| `my_python_subscriber` | `ament_python` | Python-Subscriber `data_receiver`                           |

## Verwendung

Die drei Ordner in den Ordner `src` eines Workspace kopieren, dann:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch my_tutorial_package counter.launch.py            # Publisher und Subscriber gemeinsam
ros2 launch my_tutorial_package counter.launch.py topic:=zaehler
```

## Lifecycle-Beispiel

`../lifecycle_example/lane_detector.py` ist ein verwalteter Knoten (`LifecycleNode`) ohne eigenes Paket:

```bash
source /opt/ros/jazzy/setup.bash
python3 lane_detector.py                          # Terminal 1
ros2 lifecycle set /lane_detector configure       # Terminal 2
ros2 lifecycle set /lane_detector activate
ros2 topic echo /lane
```
