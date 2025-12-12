# Point Cloud Processing Pipeline

Ein ROS2-Paket für die Verarbeitung von Point Clouds mit Boden-Erkennung und Hindernisklassifikation.

## Übersicht

Dieses Projekt implementiert eine modulare Pipeline zur Bearbeitung von Point Cloud-Daten. Es nutzt RANSAC-basierte Algorithmen zur robusten Trennung von Bodenpunkten und Hindernissen.

## Nodes

### 1. Range Filter Node (`range_filter_node.py`)

**Zweck:** Filtert Points in einer Point Cloud basierend auf ihrer Entfernung zum Sensor.

**Parameter:**
- `min_dist` (default: 0.5): Mindestentfernung in Metern
- `max_dist` (default: 8.0): Maximale Entfernung in Metern  
- `input_topic`: Input Point Cloud Topic

**Funktionsweise:**
- Empfängt Point Cloud-Daten
- Entfernt alle Points, die außerhalb des definierten Entfernungsbereichs liegen
- Publiziert die gefilterte Point Cloud

**Topics:**
- **Input:** `/zed2i_front/zed_node_front/point_cloud/cloud_registered` (PointCloud2)
- **Output:** `/filtered_cloud` (PointCloud2)

### 2. Ground Detection Node (`ground_detection_node.py`)

**Zweck:** Erkennt Bodenpunkte und trennt sie von Hindernissen mittels RANSAC-Ebenenfit.

**Parameter:**
- `ground_threshold` (default: 0.05): Abstandsschwelle zur Bodenebene in Metern
- `min_ground_points` (default: 100): Mindestanzahl Punkte für gültige Bodenerkennung
- `input_topic`: Input Point Cloud Topic

**Funktionsweise:**
- Verwendet RANSAC-Algorithmus für robuste Ebenenfit
- Klassifiziert Points als Boden oder Hindernis basierend auf Abstand zur gefitteten Ebene
- Publiziert getrennte Point Clouds für Boden und Hindernisse

**Topics:**
- **Input:** `/filtered_cloud` (PointCloud2)
- **Output:** `/ground_cloud` (PointCloud2), `/obstacle_cloud` (PointCloud2)

## RANSAC-Algorithmus Erklärung

### Was ist RANSAC?

**RANSAC** (RANdom SAmple Consensus) ist ein iterativer Algorithmus zur robusten Schätzung von Modellparametern bei verrauschten Daten mit Outliers.

### Funktionsweise im Detail

#### 1. **Problem bei traditionellen Methoden**
```
Normale Least-Squares Regression:
- Empfindlich gegenüber Outliers
- Ein einzelner falscher Punkt kann das gesamte Modell verfälschen
- Bei Point Clouds: Objekte auf dem Boden würden die Bodenebene verzerren
```

#### 2. **RANSAC-Lösung**
```python
# Pseudo-Code für RANSAC Ebenenfit:
for iteration in range(max_trials):
    # 1. Zufällige Stichprobe
    sample_points = randomly_select(min_samples)  # z.B. 50 Punkte
    
    # 2. Modell fitten
    plane_model = fit_plane(sample_points)
    
    # 3. Inliers zählen
    distances = calculate_distances_to_plane(all_points, plane_model)
    inliers = points_where(distances < threshold)
    
    # 4. Bestes Modell speichern
    if len(inliers) > best_inlier_count:
        best_model = plane_model
        best_inliers = inliers
```

#### 3. **Anwendung in der Ground Detection**

**Schritt 1: Datenvorverarbeitung**
```python
# Point Cloud Format: [X, Y, Z]
# X,Z = horizontale Koordinaten
# Y = Höhenkoordinate (vertikal)
X = points[:, [0, 2]]  # Eingabefeatures für Ebenenfit
y = points[:, 1]       # Zielvariable (Höhe)
```

**Schritt 2: RANSAC-Parameter**
```python
ransac = RANSACRegressor(
    residual_threshold=0.05,    # 5cm Toleranz zur Ebene
    min_samples=50,             # Mindestens 50 Punkte für Ebene
    max_trials=100,             # Maximal 100 Iterationen
    random_state=42             # Reproduzierbare Ergebnisse
)
```

**Schritt 3: Robust Plane Fitting**
```python
# Fit: y = a*x + b*z + c
# Findet automatisch die beste Ebene durch RANSAC
ransac.fit(X, y)

# Vorhersage für alle Punkte
predicted_heights = ransac.predict(X_all)

# Klassifikation basierend auf Abstand
distances = abs(actual_heights - predicted_heights)
ground_mask = distances < threshold
```

### Warum RANSAC für Ground Detection?

#### **Vorteile:**

1. **Robustheit gegen Outliers**
   - Objekte auf dem Boden (Steine, Müll) verfälschen die Bodenebene nicht
   - Funktioniert auch bei unvollständigen Bodendaten

2. **Automatische Inlier-Detection**
   - Erkennt automatisch, welche Punkte zum Boden gehören
   - Keine manuelle Schwellwertabstimmung für verschiedene Szenen nötig

3. **Skalierbarkeit**
   - Funktioniert bei unterschiedlichen Bodenneigungen
   - Adaptiert sich an geneigte Oberflächen

#### **Parameter-Tuning:**

- **`residual_threshold`**: Kleinere Werte = striktere Bodenerkennung
- **`min_samples`**: Mehr Samples = stabilere aber langsamere Fits  
- **`max_trials`**: Mehr Versuche = bessere Ergebnisse bei komplexen Szenen

#### **Typische Anwendungsszenarien:**

```
✅ Outdoor-Robotik: Unebener Boden mit Steinen, Löchern
✅ Indoor-Navigation: Möbel auf dem Boden  
✅ Autonome Fahrzeuge: Straße mit parkenden Autos
✅ Industrieumgebung: Produktionshalle mit Maschinen
```

### Algorithmus-Vergleich

| Methode | Robustheit | Performance | Genauigkeit |
|---------|------------|-------------|-------------|
| Least Squares | ❌ Niedrig | ✅ Schnell | ❌ Bei Outliers schlecht |
| RANSAC | ✅ Hoch | ⚡ Mittel | ✅ Sehr gut |
| M-Estimators | ✅ Gut | ❌ Langsam | ✅ Gut |

## Installation und Build

```bash
# Abhängigkeiten installieren
pip install scikit-learn

# Build
cd /path/to/your/ros2_workspace
colcon build

# Setup sourcing
source install/setup.bash
```

## Verwendung

### Starten der kompletten Pipeline

```bash
ros2 launch cloud_processing cloud_pipeline.launch.py
```

Diese Launch-Datei startet:
- Range Filter Node
- Ground Detection Node  
- RViz mit vorkonfigurierter Visualisierung

### Parameter anpassen

```bash
# Ground Detection mit angepassten Parametern
ros2 run cloud_processing ground_detection_node --ros-args \
    -p ground_threshold:=0.03 \
    -p min_ground_points:=200
```

## Visualisierung

Das Projekt enthält eine vorkonfigurierte RViz-Konfiguration, die folgende Displays zeigt:

1. **Original Point Cloud** (Input)
2. **Filtered Point Cloud** (nach Range Filter)
3. **Ground Points** (erkannte Bodenpunkte - grün)
4. **Obstacle Points** (Hindernisse - rot)

## Pipeline-Architektur

```
Input Point Cloud 
      ↓
Range Filter Node
      ↓
Filtered Point Cloud
      ↓  
Ground Detection Node (RANSAC)
      ↓
Ground Points + Obstacle Points
```

## Anwendungsfälle

- **Autonome Navigation:** Sichere Pfadplanung durch Boden-Hindernis-Trennung
- **Obstacle Avoidance:** Echtzeitdetektion von Hindernissen
- **Mapping:** Erstellung von Occupancy Grids
- **Landwirtschaft:** Navigation zwischen Pflanzenreihen
- **Baustellenrobotik:** Navigation auf unebenen Oberflächen

## Abhängigkeiten

- ROS2 (getestet mit Jazzy)
- sensor_msgs
- numpy
- scikit-learn (für RANSAC)
- rclpy

## Troubleshooting

**Problem:** Ground Detection funktioniert nicht
**Lösung:** 
- Überprüfen Sie `ground_threshold` (zu strikt?)
- Erhöhen Sie `min_ground_points` bei dichten Point Clouds
- Vergewissern Sie sich, dass die Point Cloud genügend Bodenpunkte enthält

**Problem:** Zu viele False Positives bei Hindernissen
**Lösung:** Verringern Sie `ground_threshold` (z.B. auf 0.03)

**Problem:** Performance-Probleme
**Lösung:** Downsampling wird automatisch angewendet (jeder 3. Punkt)