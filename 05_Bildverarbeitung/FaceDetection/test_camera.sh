#!/bin/bash
# Test-Skript für Kamera-Zugriff

echo "================================================"
echo "Kamera-Test für Face Detection"
echo "================================================"
echo ""

# Prüfe verfügbare Kameras
echo "1. Verfügbare Video-Geräte:"
v4l2-ctl --list-devices

echo ""
echo "2. Detaillierte Info für /dev/video0:"
if [ -e /dev/video0 ]; then
    v4l2-ctl --device=/dev/video0 --all | head -20
else
    echo "  ✗ /dev/video0 nicht gefunden!"
fi

echo ""
echo "3. Berechtigungen:"
ls -la /dev/video* 2>/dev/null || echo "  Keine Video-Geräte gefunden"

echo ""
echo "4. Aktuelle Gruppen des Users:"
groups

echo ""
echo "================================================"
echo "Hinweise:"
echo "================================================"
if groups | grep -q video; then
    echo "✓ User ist in 'video' Gruppe"
else
    echo "✗ User NICHT in 'video' Gruppe!"
    echo "  Lösung: sudo usermod -a -G video $USER"
    echo "  Dann neu einloggen!"
fi

echo ""
echo "Kamera testen mit:"
echo "  ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0"
