#!/bin/bash
# Build-Skript für Face Detection Workspace

set -e

echo "================================================"
echo "Building Face Detection ROS2 Workspace"
echo "================================================"

# Prüfe ROS2-Installation
if [ -z "$ROS_DISTRO" ]; then
    echo "ROS2 nicht gefunden! Sourcing /opt/ros/jazzy/setup.bash"
    source /opt/ros/jazzy/setup.bash
fi

echo "ROS2 Distribution: $ROS_DISTRO"

# Workspace-Verzeichnis
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

echo "Workspace: $WORKSPACE_DIR"

# Abhängigkeiten installieren (optional)
echo ""
echo "Prüfe Abhängigkeiten..."
read -p "Abhängigkeiten installieren? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo apt update
    sudo apt install -y \
        ros-${ROS_DISTRO}-v4l2-camera \
        ros-${ROS_DISTRO}-rqt-image-view \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-rosbag2-storage-mcap \
        v4l-utils \
        python3-pip

    # OpenCV über apt statt pip installieren (vermeidet externally-managed-environment)
    sudo apt install -y python3-opencv || echo "OpenCV bereits installiert"
fi

# Workspace bauen
echo ""
echo "Building workspace..."
colcon build --symlink-install

# Build-Status prüfen
if [ $? -eq 0 ]; then
    echo ""
    echo "================================================"
    echo "✓ Build erfolgreich!"
    echo "================================================"
    echo ""
    echo "Nächste Schritte:"
    echo "  1. Source workspace:"
    echo "     source install/setup.bash"
    echo ""
    echo "  2. Starte System:"
    echo "     ros2 launch face_detector face_detection.launch.py"
    echo ""
    echo "  3. Oder teste einzelne Nodes:"
    echo "     ros2 run face_detector face_detector_node"
    echo ""
else
    echo ""
    echo "================================================"
    echo "✗ Build fehlgeschlagen!"
    echo "================================================"
    exit 1
fi
