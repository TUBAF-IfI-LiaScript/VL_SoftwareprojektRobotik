# Lifecycle-Beispiel zur Vorlesung Navigation

_Autoren: Sebastian Zug & Claude Code (Opus 5.5) mit LiaScript Teaching-Agent — getestet mit ROS 2 Jazzy, Oktober 2026_

`lane_detector.py` ist ein verwalteter Knoten (`LifecycleNode`), der nur im Zustand `Active` sendet. Er läuft ohne eigenes Paket:

```bash
source /opt/ros/jazzy/setup.bash
python3 lane_detector.py                          # Terminal 1
ros2 lifecycle set /lane_detector configure       # Terminal 2
ros2 lifecycle set /lane_detector activate
ros2 topic echo /lane
```
