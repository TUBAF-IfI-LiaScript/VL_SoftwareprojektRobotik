#!/usr/bin/env python3
"""
YOLOv8 Objekterkennung - Einfaches Beispiel
============================================

Dieses Skript demonstriert die Verwendung eines vortrainierten YOLOv8-Modells
zur Erkennung von Objekten in Bildern oder Webcam-Streams.

Installation:
    pip install ultralytics opencv-python

Verwendung:
    # Mit Webcam:
    python detect_objects.py

    # Mit Bilddatei:
    python detect_objects.py --image pfad/zum/bild.jpg

    # Mit anderem Modell (n, s, m, l, x):
    python detect_objects.py --model yolov8s.pt
"""

import argparse
import cv2
from ultralytics import YOLO


def detect_from_image(model: YOLO, image_path: str) -> None:
    """Erkennt Objekte in einem einzelnen Bild."""

    # Bild laden
    image = cv2.imread(image_path)
    if image is None:
        print(f"Fehler: Bild '{image_path}' konnte nicht geladen werden.")
        return

    # Inferenz durchführen
    results = model(image)

    # Ergebnisse anzeigen
    for result in results:
        # Annotiertes Bild erstellen
        annotated = result.plot()

        # Erkannte Objekte ausgeben
        print("\nErkannte Objekte:")
        print("-" * 50)
        for box in result.boxes:
            cls_id = int(box.cls[0])
            cls_name = model.names[cls_id]
            confidence = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            print(f"  {cls_name}: {confidence:.2%} @ [{x1}, {y1}, {x2}, {y2}]")

        # Bild anzeigen
        cv2.imshow("YOLOv8 Detection", annotated)
        cv2.waitKey(0)

    cv2.destroyAllWindows()


def detect_from_webcam(model: YOLO, camera_id: int = 0) -> None:
    """Erkennt Objekte in einem Live-Webcam-Stream."""

    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Fehler: Kamera {camera_id} konnte nicht geöffnet werden.")
        return

    print("Webcam gestartet. Drücke 'q' zum Beenden.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Fehler beim Lesen des Frames.")
            break

        # Inferenz durchführen
        results = model(frame, verbose=False)

        # Annotiertes Bild erstellen
        annotated = results[0].plot()

        # FPS und Objektanzahl anzeigen
        num_objects = len(results[0].boxes)
        cv2.putText(
            annotated,
            f"Objekte: {num_objects}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )

        cv2.imshow("YOLOv8 Live Detection", annotated)

        # 'q' zum Beenden
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="YOLOv8 Objekterkennung"
    )
    parser.add_argument(
        "--image",
        type=str,
        help="Pfad zum Eingabebild (ohne = Webcam)"
    )
    parser.add_argument(
        "--model",
        type=str,
        default="yolov8n.pt",
        help="YOLOv8 Modell (default: yolov8n.pt)"
    )
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="Kamera-ID für Webcam (default: 0)"
    )
    args = parser.parse_args()

    # Modell laden (wird automatisch heruntergeladen falls nicht vorhanden)
    print(f"Lade Modell: {args.model}")
    model = YOLO(args.model)

    # Verfügbare Klassen anzeigen
    print(f"\nDas Modell kann {len(model.names)} Klassen erkennen:")
    print(", ".join(list(model.names.values())[:10]) + ", ...")

    if args.image:
        detect_from_image(model, args.image)
    else:
        detect_from_webcam(model, args.camera)


if __name__ == "__main__":
    main()
