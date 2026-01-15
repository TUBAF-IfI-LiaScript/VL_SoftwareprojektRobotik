#!/usr/bin/env python3
"""
Analyse-Skript für Übung 2: YOLO vs. ZED Vergleich

AUFGABE 3: Erstellen Sie einen Vergleichsplot der Detektionsergebnisse.

Dieses Skript soll:
1. Die YOLO- und ZED-CSV-Dateien einlesen
2. Die Timestamps matchen (YOLO hat weniger Frames als ZED)
3. Einen Step-Plot erstellen, der beide Detektoren vergleicht

Verwendung:
    python3 analyze_results.py
"""

import csv
import os

# Matplotlib Import
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("FEHLER: matplotlib nicht installiert!")
    print("Installieren mit: pip install matplotlib")


def load_csv(csv_path: str, count_field: str) -> list:
    """
    Lade CSV-Datei und extrahiere Timestamps und Counts.

    AUFGABE 3.1: Implementieren Sie das Laden der CSV-Datei.

    Args:
        csv_path: Pfad zur CSV-Datei
        count_field: Name der Spalte mit der Personenanzahl
                    ('yolo_count' oder 'zed_person_count')

    Returns:
        Liste von Dictionaries mit 'timestamp' und 'count'
        Leere Liste wenn Datei nicht existiert
    """
    if not os.path.exists(csv_path):
        return []

    results = []

    # =========================================================================
    # TODO: CSV-Datei laden
    # =========================================================================
    # Schritte:
    # 1. Öffnen Sie die Datei mit open(csv_path, 'r')
    # 2. Erstellen Sie einen csv.DictReader
    # 3. Für jede Zeile:
    #    - Extrahieren Sie den timestamp (Spalte 'timestamp')
    #    - Extrahieren Sie den count (Spalte count_field)
    #    - Fügen Sie ein Dictionary zur Liste hinzu:
    #      {'timestamp': float(...), 'count': int(...)}
    #
    # Beispiel:
    #   with open(csv_path, 'r') as f:
    #       reader = csv.DictReader(f)
    #       for row in reader:
    #           results.append({
    #               'timestamp': float(row['timestamp']),
    #               'count': int(row[count_field])
    #           })
    #
    # 4. Sortieren Sie die Ergebnisse nach Timestamp:
    #    results.sort(key=lambda x: x['timestamp'])
    # =========================================================================

    return results


def match_timestamps(yolo_data: list, zed_data: list, tolerance: float = 0.1) -> list:
    """
    Matche YOLO- und ZED-Daten anhand der Timestamps.

    YOLO verarbeitet weniger Frames als ZED (wegen langsamer Inferenz).
    Für jeden YOLO-Datenpunkt suchen wir den nächsten ZED-Datenpunkt.

    AUFGABE 3.2: Implementieren Sie das Timestamp-Matching.

    Args:
        yolo_data: Liste mit YOLO-Ergebnissen
        zed_data: Liste mit ZED-Ergebnissen
        tolerance: Maximale Zeitdifferenz in Sekunden (default: 100ms)

    Returns:
        Liste von Dictionaries mit:
        - 'time': Timestamp
        - 'yolo': YOLO-Count
        - 'zed': ZED-Count (oder None wenn kein Match)
    """
    # Wenn keine ZED-Daten: Nur YOLO zurückgeben
    if not zed_data:
        return [{'time': y['timestamp'], 'yolo': y['count'], 'zed': None}
                for y in yolo_data]

    matched = []

    # =========================================================================
    # TODO: Timestamp-Matching implementieren
    # =========================================================================
    # Algorithmus:
    # Für jeden YOLO-Datenpunkt:
    #   1. Suche den ZED-Datenpunkt mit dem kleinsten Zeitabstand
    #   2. Wenn Abstand <= tolerance: Match gefunden
    #   3. Sonst: zed = None
    #
    # Einfache Variante (funktioniert, aber nicht optimal):
    #   for yolo in yolo_data:
    #       yolo_ts = yolo['timestamp']
    #       best_match = None
    #       best_diff = float('inf')
    #
    #       for zed in zed_data:
    #           diff = abs(yolo_ts - zed['timestamp'])
    #           if diff < best_diff:
    #               best_diff = diff
    #               best_match = zed
    #
    #       zed_count = best_match['count'] if best_match and best_diff <= tolerance else None
    #       matched.append({'time': yolo_ts, 'yolo': yolo['count'], 'zed': zed_count})
    #
    # Effizientere Variante (optional):
    # Da beide Listen zeitlich sortiert sind, kann man sich den
    # letzten Index merken und ab dort weitersuchen.
    # =========================================================================

    return matched


def create_plot(results: list, output_path: str):
    """
    Erstelle Step-Plot und speichere als PNG.

    AUFGABE 3.3: Implementieren Sie die Visualisierung.

    Args:
        results: Liste von gematchten Ergebnissen
        output_path: Pfad für die Ausgabedatei (z.B. 'results/comparison_plot.png')

    Hinweise:
        - Verwenden Sie ax.step() für Step-Plots (diskrete Werte)
        - Parameter where='post' zeigt den Wert bis zum nächsten Zeitpunkt
        - YOLO als durchgezogene Linie, ZED als gestrichelte Linie
    """
    if not MATPLOTLIB_AVAILABLE:
        print("Plot-Erstellung übersprungen (matplotlib nicht verfügbar)")
        return

    # =========================================================================
    # TODO: Plot erstellen
    # =========================================================================
    # Schritte:
    #
    # 1. Daten vorbereiten:
    #    t_start = results[0]['time']
    #    time_rel = [r['time'] - t_start for r in results]  # Relative Zeit
    #    yolo_counts = [r['yolo'] for r in results]
    #    zed_counts = [r['zed'] for r in results]
    #
    # 2. Figure erstellen:
    #    fig, ax = plt.subplots(1, 1, figsize=(12, 4))
    #
    # 3. YOLO-Linie zeichnen (Step-Plot):
    #    ax.step(time_rel, yolo_counts, 'b-', label='YOLO',
    #            linewidth=1.5, alpha=0.8, where='post')
    #
    # 4. ZED-Linie zeichnen (wenn Daten vorhanden):
    #    if any(z is not None for z in zed_counts):
    #        ax.step(time_rel, zed_counts, 'r--', label='ZED',
    #                linewidth=1.5, alpha=0.8, where='post')
    #
    # 5. Achsenbeschriftung und Formatierung:
    #    ax.set_xlabel('Zeit [s]')
    #    ax.set_ylabel('Anzahl Personen')
    #    ax.set_title('Personendetektion über Zeit: YOLO vs. ZED')
    #    ax.legend(loc='upper right')
    #    ax.grid(True, alpha=0.3)
    #    ax.set_ylim(bottom=0)
    #
    # 6. Speichern und schließen:
    #    plt.tight_layout()
    #    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    #    plt.close()
    # =========================================================================

    print(f"Plot-Erstellung: TODO - bitte implementieren")


def main():
    """Hauptfunktion: Lade Daten, matche Timestamps, erstelle Plot."""

    # CSV-Dateien laden
    yolo_data = load_csv('results/yolo_detections.csv', 'yolo_count')
    zed_data = load_csv('results/zed_detections.csv', 'zed_person_count')

    # Prüfen ob YOLO-Daten vorhanden
    if not yolo_data:
        print("Fehler: results/yolo_detections.csv nicht gefunden oder leer")
        print("Führen Sie zuerst den YOLO-Detektor aus:")
        print("  ros2 launch person_detector detector.launch.py")
        return

    print(f"YOLO: {len(yolo_data)} Datenpunkte geladen")
    print(f"ZED:  {len(zed_data)} Datenpunkte geladen")

    # Timestamps matchen
    results = match_timestamps(yolo_data, zed_data)
    print(f"Gematchte Datenpunkte: {len(results)}")

    # Output-Verzeichnis erstellen
    os.makedirs('results', exist_ok=True)

    # Plot erstellen
    create_plot(results, 'results/comparison_plot.png')


if __name__ == '__main__':
    main()
