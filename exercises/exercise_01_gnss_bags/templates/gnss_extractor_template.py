#!/usr/bin/env python3
"""
GNSS Data Extractor - Template für Übung 1.1

Aufgabe: Extrahieren Sie GNSS-Daten aus dem ROS2-Bag und speichern Sie als CSV

Hinweise:
- Nutzen Sie rosbag2_py zum Lesen des Bags
- Verarbeiten Sie /fixposition/gnss1, /fixposition/gnss2, /fixposition/odometry_llh
- Speichern Sie timestamp, lat, lon, alt, covariance
"""

import csv
import os
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def extract_gnss_data(bag_path, topic_name, output_csv):
    """
    Extrahiert GNSS-Daten aus einem ROS2-Bag

    Args:
        bag_path: Pfad zum Bag-File
        topic_name: Name des GNSS-Topics
        output_csv: Pfad zur Ausgabe-CSV-Datei
    """

    # TODO: Bag-File öffnen
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # TODO: Topic-Typen ermitteln
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    # Prüfen, ob Topic existiert
    if topic_name not in type_map:
        print(f"Fehler: Topic {topic_name} nicht im Bag gefunden!")
        return

    # TODO: CSV-Datei vorbereiten
    data_rows = []
    start_time = None

    print(f"Extrahiere Daten von {topic_name}...")

    # TODO: Durch alle Nachrichten iterieren
    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic == topic_name:
            # Nachricht deserialisieren
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)

            # Zeitstempel (in Sekunden seit Start)
            if start_time is None:
                start_time = timestamp
            time_sec = (timestamp - start_time) / 1e9

            # TODO: GNSS-Daten extrahieren
            # msg.latitude, msg.longitude, msg.altitude
            # msg.position_covariance (9 Elemente, diagonal: [0], [4], [8])

            latitude = msg.latitude
            longitude = msg.longitude
            altitude = msg.altitude

            # Position Covariance (3x3 Matrix, diagonal)
            # Covariance-Matrix ist row-major: [0,1,2; 3,4,5; 6,7,8]
            cov_lat = msg.position_covariance[0]  # σ²_lat
            cov_lon = msg.position_covariance[4]  # σ²_lon
            cov_alt = msg.position_covariance[8]  # σ²_alt

            # Datenzeile hinzufügen
            data_rows.append([
                time_sec,
                latitude,
                longitude,
                altitude,
                cov_lat,
                cov_lon,
                cov_alt
            ])

    # TODO: CSV-Datei schreiben
    with open(output_csv, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)

        # Header
        writer.writerow(['timestamp', 'latitude', 'longitude', 'altitude',
                        'cov_lat', 'cov_lon', 'cov_alt'])

        # Daten
        writer.writerows(data_rows)

    print(f"✓ {len(data_rows)} Datenpunkte in {output_csv} gespeichert")


def main():
    """Hauptfunktion"""

    # Pfade definieren
    script_dir = Path(__file__).parent.parent
    bag_path = str(script_dir / 'data' / 'dataset.mcap')
    results_dir = script_dir / 'results'
    results_dir.mkdir(exist_ok=True)

    # TODO: Extrahieren Sie alle drei GNSS-Topics
    topics = [
        ('/fixposition/gnss1', 'gnss1_data.csv'),
        ('/fixposition/gnss2', 'gnss2_data.csv'),
        ('/fixposition/odometry_llh', 'odometry_llh_data.csv'),
    ]

    for topic_name, csv_filename in topics:
        output_path = results_dir / csv_filename
        extract_gnss_data(bag_path, topic_name, str(output_path))

    print("\n✓ Alle GNSS-Daten erfolgreich extrahiert!")


if __name__ == '__main__':
    main()
