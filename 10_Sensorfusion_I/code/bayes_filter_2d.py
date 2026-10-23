"""
2D Diskreter Bayes-Filter Simulation
=====================================

Ein Roboter bewegt sich auf einem 2D-Gitter und lokalisiert sich
anhand von Landmarken-Messungen und Bewegungskommandos.

Autor: Sebastian Zug, Georg Jäger
Vorlesung: Softwareprojekt Robotik - VL 08
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import convolve

# ============================================================================
# Weltmodell
# ============================================================================

def create_world(rows=10, cols=10, landmark_positions=None):
    """
    Erstellt eine 2D-Welt mit Landmarken.

    Args:
        rows, cols: Größe des Gitters
        landmark_positions: Liste von (row, col) Tupeln für Landmarken

    Returns:
        world: 2D-Array mit 1 an Landmarken-Positionen, sonst 0
    """
    world = np.zeros((rows, cols))

    if landmark_positions is None:
        # Standard-Landmarken
        landmark_positions = [(0, 2), (0, 7), (2, 0), (2, 9),
                              (4, 1), (4, 6), (7, 3), (7, 8),
                              (9, 0), (9, 5)]

    for (r, c) in landmark_positions:
        if 0 <= r < rows and 0 <= c < cols:
            world[r, c] = 1

    return world

# ============================================================================
# Sensormodell
# ============================================================================

def create_sensor_model(world, p_hit=0.85, p_miss=0.1):
    """
    Erstellt das Sensormodell für Landmarken-Erkennung.

    Args:
        world: 2D-Array der Welt
        p_hit: P(Landmarke erkannt | Landmarke vorhanden)
        p_miss: P(Landmarke erkannt | keine Landmarke vorhanden)

    Returns:
        likelihood_hit: Likelihood-Array wenn Landmarke gemessen
        likelihood_miss: Likelihood-Array wenn keine Landmarke gemessen
    """
    # Likelihood wenn Sensor "Landmarke" meldet
    likelihood_hit = np.where(world == 1, p_hit, p_miss)

    # Likelihood wenn Sensor "keine Landmarke" meldet
    likelihood_miss = np.where(world == 1, 1 - p_hit, 1 - p_miss)

    return likelihood_hit, likelihood_miss

def measurement_update(belief, world, measurement, p_hit=0.85, p_miss=0.1):
    """
    Update-Schritt: Aktualisiert belief basierend auf Sensormessung.

    Args:
        belief: Aktuelle Aufenthaltswahrscheinlichkeit (2D-Array)
        world: Weltmodell mit Landmarken
        measurement: True wenn Landmarke erkannt, False sonst
        p_hit, p_miss: Sensorparameter

    Returns:
        Aktualisierte belief (normalisiert)
    """
    likelihood_hit, likelihood_miss = create_sensor_model(world, p_hit, p_miss)

    if measurement:
        likelihood = likelihood_hit
    else:
        likelihood = likelihood_miss

    # Bayes-Update: posterior = prior * likelihood / normalization
    posterior = belief * likelihood
    posterior = posterior / np.sum(posterior)  # Normalisierung

    return posterior

# ============================================================================
# Bewegungsmodell
# ============================================================================

def create_motion_kernel(direction, p_correct=0.7, p_adjacent=0.1, p_stay=0.1):
    """
    Erstellt einen 3x3 Bewegungskernel für die Faltung.

    Args:
        direction: 'north', 'south', 'east', 'west' oder 'stay'
        p_correct: Wahrscheinlichkeit für korrekte Bewegung
        p_adjacent: Wahrscheinlichkeit für seitliche Abweichung
        p_stay: Wahrscheinlichkeit stehen zu bleiben

    Returns:
        3x3 Kernel-Array
    """
    kernel = np.zeros((3, 3))

    # Kernel-Indizes: [0,0]=NW, [0,1]=N, [0,2]=NE
    #                 [1,0]=W,  [1,1]=C, [1,2]=E
    #                 [2,0]=SW, [2,1]=S, [2,2]=SE

    if direction == 'north':
        kernel[0, 1] = p_correct  # Nach Norden
        kernel[0, 0] = p_adjacent  # Nordwest (Abweichung)
        kernel[0, 2] = p_adjacent  # Nordost (Abweichung)
        kernel[1, 1] = p_stay      # Stehen bleiben
    elif direction == 'south':
        kernel[2, 1] = p_correct
        kernel[2, 0] = p_adjacent
        kernel[2, 2] = p_adjacent
        kernel[1, 1] = p_stay
    elif direction == 'east':
        kernel[1, 2] = p_correct
        kernel[0, 2] = p_adjacent
        kernel[2, 2] = p_adjacent
        kernel[1, 1] = p_stay
    elif direction == 'west':
        kernel[1, 0] = p_correct
        kernel[0, 0] = p_adjacent
        kernel[2, 0] = p_adjacent
        kernel[1, 1] = p_stay
    else:  # stay
        kernel[1, 1] = 1.0

    # Normalisieren
    kernel = kernel / np.sum(kernel)
    return kernel

def motion_update(belief, direction, p_correct=0.7, p_adjacent=0.1, p_stay=0.1):
    """
    Predict-Schritt: Aktualisiert belief basierend auf Bewegungskommando.

    Args:
        belief: Aktuelle Aufenthaltswahrscheinlichkeit
        direction: Bewegungsrichtung
        p_correct, p_adjacent, p_stay: Bewegungsparameter

    Returns:
        Vorhergesagte belief nach Bewegung
    """
    kernel = create_motion_kernel(direction, p_correct, p_adjacent, p_stay)

    # Faltung mit wrap-around (zyklische Randbedingung)
    predicted = convolve(belief, kernel, mode='wrap')

    # Normalisieren (numerische Stabilität)
    predicted = predicted / np.sum(predicted)

    return predicted

# ============================================================================
# Simulation
# ============================================================================

def simulate_robot(world, true_path, sensor_accuracy=0.85):
    """
    Simuliert einen Roboter, der einem Pfad folgt.

    Args:
        world: Weltmodell
        true_path: Liste von (row, col) Positionen
        sensor_accuracy: p_hit Parameter

    Returns:
        measurements: Liste von True/False Messungen
        movements: Liste von Bewegungsrichtungen
    """
    measurements = []
    movements = []

    for i, (r, c) in enumerate(true_path):
        # Sensormessung mit Rauschen
        has_landmark = world[r, c] == 1
        if has_landmark:
            measurement = np.random.random() < sensor_accuracy
        else:
            measurement = np.random.random() < (1 - sensor_accuracy)
        measurements.append(measurement)

        # Bewegungsrichtung zum nächsten Punkt
        if i < len(true_path) - 1:
            next_r, next_c = true_path[i + 1]
            dr, dc = next_r - r, next_c - c

            if dr < 0:
                movements.append('north')
            elif dr > 0:
                movements.append('south')
            elif dc > 0:
                movements.append('east')
            elif dc < 0:
                movements.append('west')
            else:
                movements.append('stay')

    return measurements, movements

# ============================================================================
# Visualisierung
# ============================================================================

def plot_belief(belief, world, true_position=None, title="Belief", ax=None):
    """
    Visualisiert die Aufenthaltswahrscheinlichkeit als Heatmap.
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(8, 8))

    # Heatmap der Belief
    im = ax.imshow(belief, cmap='Blues', vmin=0, vmax=np.max(belief)*1.2)

    # Landmarken markieren
    landmark_positions = np.argwhere(world == 1)
    for (r, c) in landmark_positions:
        ax.plot(c, r, 'g^', markersize=15, markeredgecolor='darkgreen',
                markeredgewidth=2, label='Landmarke' if r == landmark_positions[0, 0] else '')

    # Wahre Position markieren
    if true_position is not None:
        ax.plot(true_position[1], true_position[0], 'ro', markersize=20,
                markeredgecolor='darkred', markeredgewidth=3, label='Wahre Position')

    # Geschätzte Position (Maximum der Belief)
    est_pos = np.unravel_index(np.argmax(belief), belief.shape)
    ax.plot(est_pos[1], est_pos[0], 'bx', markersize=15, markeredgewidth=3,
            label='Geschätzte Position')

    ax.set_title(title)
    ax.set_xlabel('Spalte')
    ax.set_ylabel('Zeile')
    ax.legend(loc='upper right')

    # Gitterlinien
    ax.set_xticks(np.arange(-0.5, belief.shape[1], 1), minor=True)
    ax.set_yticks(np.arange(-0.5, belief.shape[0], 1), minor=True)
    ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5)

    return im

def run_filter_visualization(world, true_path, save_path=None):
    """
    Führt den Bayes-Filter aus und visualisiert jeden Schritt.
    """
    rows, cols = world.shape

    # Initiale Belief: Gleichverteilung
    belief = np.ones((rows, cols)) / (rows * cols)

    # Simuliere Messungen und Bewegungen
    measurements, movements = simulate_robot(world, true_path)

    # Anzahl der Schritte
    n_steps = len(true_path)

    # Figure für Animation
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    axes = axes.flatten()

    beliefs_to_show = [0, n_steps//4, n_steps//2, 3*n_steps//4, n_steps-1]

    step_idx = 0
    for i in range(n_steps):
        # Measurement Update
        belief = measurement_update(belief, world, measurements[i])

        # Visualisierung an ausgewählten Schritten
        if i in beliefs_to_show and step_idx < len(axes):
            plot_belief(belief, world, true_path[i],
                       f"Schritt {i+1}: Nach Messung", axes[step_idx])
            step_idx += 1

        # Motion Update (wenn nicht letzter Schritt)
        if i < len(movements):
            belief = motion_update(belief, movements[i])

    # Finaler Zustand
    if step_idx < len(axes):
        plot_belief(belief, world, true_path[-1],
                   f"Final: Schritt {n_steps}", axes[step_idx])

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')

    plt.show()

    return belief

# ============================================================================
# Hauptprogramm
# ============================================================================

if __name__ == "__main__":
    # Welt erstellen
    world = create_world(10, 10)

    # Pfad definieren (Roboter startet bei (5, 2) und bewegt sich)
    true_path = [
        (5, 2), (5, 3), (5, 4), (5, 5), (5, 6),  # Nach Osten
        (4, 6), (3, 6), (2, 6),                   # Nach Norden
        (2, 7), (2, 8), (2, 9),                   # Nach Osten
    ]

    print("2D Diskreter Bayes-Filter Simulation")
    print("=" * 40)
    print(f"Weltgröße: {world.shape}")
    print(f"Anzahl Landmarken: {int(np.sum(world))}")
    print(f"Pfadlänge: {len(true_path)} Schritte")
    print()

    # Filter ausführen
    final_belief = run_filter_visualization(world, true_path)

    # Ergebnis auswerten
    est_pos = np.unravel_index(np.argmax(final_belief), final_belief.shape)
    true_pos = true_path[-1]

    print(f"Wahre Endposition: {true_pos}")
    print(f"Geschätzte Position: {est_pos}")
    print(f"Maximale Belief: {np.max(final_belief):.4f}")
