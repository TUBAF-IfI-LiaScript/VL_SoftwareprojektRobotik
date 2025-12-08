#!/usr/bin/env python3
"""
GNSS Trajectory Plotter - Template für Übung 1.2

Aufgabe: Plotten Sie die GNSS-Trajektorien und berechnen Sie Streckenlängen

Hinweise:
- Lesen Sie die CSV-Dateien aus Aufgabe 1.1
- Konvertieren Sie Lat/Lon zu Metern (relative Koordinaten)
- Plotten Sie alle Trajektorien in einem Diagramm
- Berechnen Sie die Gesamtstrecke
"""

import csv
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def latlon_to_meters(lat, lon, lat0, lon0):
    """
    Konvertiert Lat/Lon zu relativen Metern (Flat-Earth-Approximation)

    Args:
        lat, lon: Aktuelle Position (Grad)
        lat0, lon0: Referenzposition (Grad)

    Returns:
        x, y: Position in Metern relativ zu (lat0, lon0)
    """
    R = 6371000  # Erdradius in Metern

    # TODO: Implementieren Sie die Konvertierung
    # x = (lon - lon0) * cos(lat0) * R * π/180
    # y = (lat - lat0) * R * π/180

    x = (lon - lon0) * np.cos(np.radians(lat0)) * R * np.pi / 180
    y = (lat - lat0) * R * np.pi / 180

    return x, y


def calculate_distance(x, y):
    """
    Berechnet die Gesamtstrecke einer Trajektorie

    Args:
        x, y: Arrays mit Koordinaten

    Returns:
        total_distance: Gesamtstrecke in Metern
    """
    # TODO: Berechnen Sie die euklidische Distanz zwischen aufeinanderfolgenden Punkten
    # dx = diff(x), dy = diff(y)
    # distances = sqrt(dx² + dy²)
    # total = sum(distances)

    dx = np.diff(x)
    dy = np.diff(y)
    distances = np.sqrt(dx**2 + dy**2)
    return np.sum(distances)


def load_gnss_csv(csv_path):
    """
    Lädt GNSS-Daten aus CSV

    Returns:
        timestamps, latitudes, longitudes, altitudes
    """
    timestamps = []
    latitudes = []
    longitudes = []
    altitudes = []

    with open(csv_path, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            timestamps.append(float(row['timestamp']))
            latitudes.append(float(row['latitude']))
            longitudes.append(float(row['longitude']))
            altitudes.append(float(row['altitude']))

    return (np.array(timestamps), np.array(latitudes),
            np.array(longitudes), np.array(altitudes))


def plot_trajectories(trajectories, output_path):
    """
    Plottet mehrere GNSS-Trajektorien

    Args:
        trajectories: Dict mit {name: (x, y)} Paaren
        output_path: Pfad für Ausgabe-PNG
    """
    plt.figure(figsize=(12, 10))

    # TODO: Plotten Sie alle Trajektorien
    colors = ['blue', 'red', 'green', 'orange']

    for i, (name, (x, y)) in enumerate(trajectories.items()):
        plt.plot(x, y, label=name, color=colors[i % len(colors)],
                linewidth=2, alpha=0.7)

        # Markiere Startpunkt
        plt.plot(x[0], y[0], 'o', color=colors[i % len(colors)],
                markersize=10, label=f'{name} Start')

    # TODO: Diagramm beschriften
    plt.xlabel('X [m]', fontsize=12)
    plt.ylabel('Y [m]', fontsize=12)
    plt.title('GNSS-Trajektorie Vergleich', fontsize=14, fontweight='bold')
    plt.legend(loc='best')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')  # Gleiche Skalierung für x und y

    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    print(f"✓ Trajektorien-Plot gespeichert: {output_path}")
    plt.close()


def main():
    """Hauptfunktion"""

    # Pfade
    script_dir = Path(__file__).parent.parent
    results_dir = script_dir / 'results'

    # TODO: Laden Sie die CSV-Dateien
    csv_files = {
        'GNSS1': results_dir / 'gnss1_data.csv',
        'GNSS2': results_dir / 'gnss2_data.csv',
        'Odometry LLH': results_dir / 'odometry_llh_data.csv',
    }

    # Referenzposition (erste Position von GNSS1)
    _, lats_ref, lons_ref, _ = load_gnss_csv(csv_files['GNSS1'])
    lat0, lon0 = lats_ref[0], lons_ref[0]

    print(f"Referenzposition: {lat0:.6f}°N, {lon0:.6f}°E")

    # Konvertiere alle Trajektorien
    trajectories = {}
    distances = {}

    for name, csv_path in csv_files.items():
        if not csv_path.exists():
            print(f"Warnung: {csv_path} nicht gefunden, überspringe...")
            continue

        # Lade Daten
        timestamps, lats, lons, alts = load_gnss_csv(csv_path)

        # TODO: Konvertiere zu Metern
        x, y = latlon_to_meters(lats, lons, lat0, lon0)

        trajectories[name] = (x, y)

        # TODO: Berechne Streckenlänge
        distance = calculate_distance(x, y)
        distances[name] = distance

        print(f"{name}: {len(timestamps)} Punkte, {distance:.2f} m")

    # TODO: Plotten Sie die Trajektorien
    plot_trajectories(trajectories, results_dir / 'task_1_2_gnss_trajectory.png')

    # TODO: Speichern Sie die Streckenlängen
    with open(results_dir / 'task_1_2_distances.txt', 'w') as f:
        f.write("GNSS-Trajektorie Streckenlängen\n")
        f.write("=" * 40 + "\n\n")
        for name, distance in distances.items():
            f.write(f"{name}: {distance:.2f} m\n")

    print(f"\n✓ Trajektorien-Analyse abgeschlossen!")


if __name__ == '__main__':
    main()
