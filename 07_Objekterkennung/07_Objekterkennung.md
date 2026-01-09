<!--

author:   Sebastian Zug & Claude.ai
email:    sebastian.zug@informatik.tu-freiberg.de & Georg.Jaeger@informatik.tu-freiberg.de
version:  0.0.1
language: de
narrator: Deutsch Female

import: https://github.com/liascript/CodeRunner
        https://raw.githubusercontent.com/liascript-templates/plantUML/master/README.md
        https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/config.md

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/07_Objekterkennung/07_Objekterkennung.md)

# Objekterkennung und Tracking

| Parameter            | Kursinformationen                                                                                                     |
| -------------------- | --------------------------------------------------------------------------------------------------------------------- |
| **Veranstaltung:**   | @config.lecture                                                                                                       |
| **Semester**         | @config.semester                                                                                                      |
| **Hochschule:**      | `Technische Universität Freiberg`                                                                                     |
| **Inhalte:**         | `Objekterkennung mit Features und Deep Learning, Tracking, 3D-Objekterkennung`                                        |
| **Link auf GitHub:** | https://github.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/blob/master/07_Objekterkennung/07_Objekterkennung.md |
| **Autoren**          | @author                                                                                                               |

![](https://media.giphy.com/media/3o7btPCcdNniyf0ArS/giphy.gif)

--------------------------------------------------------------------------------

**Zielstellung der heutigen Veranstaltung**

+ Verständnis feature-basierter Objekterkennung (ORB, AKAZE)
+ Deep Learning für Object Detection (YOLOv8) - **Vorbereitung für Übung 2**
+ Object Tracking über Bildsequenzen (DeepSORT)
+ 3D-Objekterkennung in Punktwolken (RANSAC, Clustering)
+ Integration in ROS 2 mit vision_msgs
+ **Praktische Anwendung: Personendetektion mit YOLOv8 + Stereo-Positionsschätzung**

--------------------------------------------------------------------------------

## Motivation: Warum Objekterkennung?

    --{{0}}--
In den letzten beiden Vorlesungen haben wir gelernt, wie Kameras Bilder aufnehmen und wie wir aus Stereo-Bildern Tiefeninformationen gewinnen können. Heute geht es um die nächste Stufe: Wie erkennen Roboter konkrete Objekte in ihrer Umgebung?

> Wir möchten nicht nur sehen, sondern auch verstehen. Das bedeutet, dass wir Objekte identifizieren, lokalisieren und ihr Verhalten vorhersagen müssen, um darauf reagieren zu können.

```ascii
Kamerabild          Objekt        Verhalten
   ┌─────┐         ┌─────┐        ┌──────┐
   │ ░░▓ │  ─────> │  🚶 │ ─────> │ Stop │
   │ ▓▓░ │         │ @3m │        │  !   │
   │ ░▓▓ │         └─────┘        └──────┘
   └─────┘                                                                                         .
```

    --{{0}}--
Ein Roboter muss nicht nur Pixel sehen, sondern verstehen: Was ist das? Wo ist es? Wie bewegt es sich? Diese Fähigkeiten sind essentiell für autonome Navigation, Manipulation und Mensch-Roboter-Interaktion.

**Anwendungsbeispiele**

| Anwendung                     | Aufgabe                              | Beispiel                                   |
| ----------------------------- | ------------------------------------ | ------------------------------------------ |
| **Autonomes Fahren**          | Fußgänger, Fahrzeuge, Schilder       | Tesla Autopilot, Waymo                      |
| **Mobile Robotik**            | Hinderniserkennung, Person-Following | Serviceroboter in Hotels                    |
| **Industrielle Manipulation** | Objekterkennung für Pick-and-Place   | Bin-Picking in Lagerhallen                  |
| **Drohnen**                   | Landing Pad Detection                | Autonome Landung                            |
| **Soziale Robotik**           | Gesichtserkennung, Gestenerkennung   | Pepper, NAO                                 |

https://github.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/blob/master/00_Einfuehrung/images/ROSE2024_Chemnitz.pdf

> **Übung 2 Kontext**: Wir werden YOLOv8 verwenden, um Personen in Kamerabildern zu erkennen und deren 3D-Position mit Stereo-Vision zu bestimmen!

### Begriffsdefinitionen

**Objektbezogene Perspective:**

| Begriff                              | Fragestellung               | Ergebnis                  | Beispiel                                       |
| ------------------------------------ | --------------------------- | ------------------------- | ---------------------------------------------- |
| **Detection** (Detektion)            | Wo ist ein Objekt?          | Bounding Box + Klasse     | "Person bei (320, 240), 80×150 Pixel"          |
| **Classification** (Klassifikation)  | Was ist das Objekt?         | Klasse + Confidence       | "Person mit 95% Wahrscheinlichkeit"            |
| **Identification** (Identifikation)  | Wer/welches Individuum?     | ID innerhalb einer Klasse | "Das ist Max Müller"                           |
| **Tracking** (Verfolgung)            | Wie bewegt sich das Objekt? | Trajektorie über Zeit     | "Person #42 bewegt sich mit 1 m/s nach rechts" |

```ascii
Klassifikation vs. Identifikation:

Klassifikation:                    Identifikation:
"Was ist es?" (Kategorie)          "Wer ist es?" (Individuum)

  🚶  🚶  🚶                         🚶  🚶  🚶
   ↓   ↓   ↓                          ↓   ↓   ↓
 Person Person Person               Max  Anna  Tom
 (alle gleiche Klasse)              (unterschiedliche IDs)                                                  .
```

**Pixelbasierte Perspektive (Segmentation):**

| Begriff                   | Fragestellung                           | Ergebnis       | Beispiel               |
| ------------------------- | --------------------------------------- | -------------- | ---------------------- |
| **Instance Segmentation** | Welche Pixel gehören zu welchem Objekt? | Maske pro Objekt | Mask R-CNN           |
| **Semantic Segmentation** | Welcher Klasse gehört jedes Pixel?      | Klassenmaske   | Straße, Gehweg, Himmel |



## Abgrenzung: Traditionelle vs. Deep Learning Methoden

In dieser Vorlesung behandeln wir **beide Ansätze** der Objekterkennung:

| Aspekt | Traditionelle Methoden (Features) | Deep Learning (CNN, YOLO) |
|--------|-----------------------------------|---------------------------|
| **Ziel** | Geometrische Korrespondenzen | Semantische Klassifikation |
| **Anwendung** | SLAM, Visual Odometry, Stereo-Matching, Kamerakalibrierung | Objekterkennung, Autonomes Fahren |
| **Trainingsdaten** | Keine nötig | Große annotierte Datensätze |
| **Rechenleistung** | CPU ausreichend | GPU erforderlich |
| **Interpretierbarkeit** | Transparent, mathematisch fundiert | "Black Box" |

**Warum beides lernen?**

1. **Komplementäre Stärken**: Feature-Methoden liefern präzise geometrische Information (wo ist ein Punkt im 3D-Raum?), während Deep Learning semantisches Verständnis bietet (was ist das Objekt?).

2. **Hybride Systeme**: Moderne Robotik-Anwendungen kombinieren oft beide Ansätze - z.B. YOLO für Objekterkennung + ORB-Features für Tracking und Lokalisierung.

3. **Ressourcen-Constraints**: Auf eingebetteten Systemen sind klassische Methoden oft die einzige Option.

4. **Grundlagenverständnis**: Die mathematischen Konzepte hinter Feature-Detection bilden die Basis für das Verständnis moderner Architekturen.

## Feature-basierte Objekterkennung (2D)

    --{{0}}--
Traditionelle Objekterkennung basiert auf charakteristischen Merkmalen - sogenannten Features. Diese Methoden sind auch heute noch relevant, besonders für SLAM und visuelle Odometrie.

**Feature = charakteristischer, wiedererkennbarer Ausschnitt eines Bildes**

Eigenschaften guter Features:

+ **Repeatability**: Unter verschiedenen Bedingungen wiedererkennbar
+ **Distinctiveness**: Eindeutig unterscheidbar von anderen Features
+ **Locality**: Robust gegen Verdeckung
+ **Efficiency**: Schnell berechenbar
+ **Invariance**: Unabhängig von Rotation, Skalierung, Beleuchtung

| Gute Features | Schlechte Features    |
| ------------- | --------------------- |
| Ecken ✓       | Glatte Flächen ✗      |
| Kanten ✓      | Regelmäßige Muster ✗  |
| Blobs ✓       | Homogene Bereiche ✗   |

> Warum wollen wir überhaupt gleiche Features in verschiedenen Bildern finden? Weil wir so Korrespondenzen herstellen können, die für 3D-Rekonstruktion, Bewegungsschätzung und Objekterkennung essentiell sind!

### Corner Detection: Harris & Shi-Tomasi & FAST

    --{{0}}--
Ecken sind ideale Features, weil sie in zwei Richtungen starke Gradienten haben.

**Harris Corner Detector (1988)**

Idee: Suche Bereiche, wo das Bild in alle Richtungen stark variiert

**Schritt 1: Gradientenbilder berechnen**

Für jeden Pixel wird der Gradient (Helligkeitsänderung) durch Faltung des Graustufenbildes $I$ mit dem Sobel-Kernel berechnet:

$$
I_x = I * S_x \quad \text{mit} \quad S_x = \begin{bmatrix} -1 & 0 & +1 \\ -2 & 0 & +2 \\ -1 & 0 & +1 \end{bmatrix}
$$

$$
I_y = I * S_y \quad \text{mit} \quad S_y = \begin{bmatrix} -1 & -2 & -1 \\ 0 & 0 & 0 \\ +1 & +2 & +1 \end{bmatrix}
$$

Dabei ist $I$ das Eingabebild (Graustufenwerte 0-255), $*$ die Faltungsoperation und $S_x$, $S_y$ die Sobel-Kernel. Das Ergebnis sind zwei komplette Bilder $I_x$ und $I_y$ mit den Gradienten für jeden Pixel.

**Schritt 2: Struktur-Matrix M für jeden Pixel**

Für jeden Pixel $(x_0, y_0)$ wird über ein lokales Fenster W summiert:

$$
M = \sum_{(x,y) \in W} \begin{bmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{bmatrix}
$$

```ascii
Bedeutung der Matrix-Einträge:

┌─────────────────┬────────────────────────────────────┐
│ Σ Ix²           │ Stärke horizontaler Änderungen     │
│ Σ Iy²           │ Stärke vertikaler Änderungen       │
│ Σ Ix·Iy         │ Korrelation beider Richtungen      │
└─────────────────┴────────────────────────────────────┘

Beispiele:

Vertikale Kante:        Horizontale Kante:      Ecke:
    ░░██                    ░░░░                  ░░██
    ░░██                    ████                  ████

Ix: groß, Iy: klein     Ix: klein, Iy: groß    Ix: groß, Iy: groß
→ Nur eine Richtung     → Nur eine Richtung    → Beide Richtungen!
→ KANTE                 → KANTE                → ECKE ✓                                                     .
```

> Wie können wir aber die "Stärke der Ecke" beschreiben? Dafür nutzen wir die Eigenwerte der Matrix M.

**Schritt 3: Corner Response Function**

Die Eigenwerte $\lambda_1$, $\lambda_2$ von M beschreiben die **Hauptrichtungen und Stärken der Intensitätsänderung**. Geometrisch definiert M eine Ellipse, deren Halbachsen durch die Eigenwerte gegeben sind:

```ascii
Flache Region:         Kante:                 Ecke:
(beide λ klein)        (ein λ groß)           (beide λ groß)

      ·                    |                    ─┼─
    · · ·                  |                    ─┼─
      ·                    |                    ─┼─

Ellipse: winzig        Ellipse: lang/schmal   Ellipse: kreisförmig
λ₁ ≈ λ₂ ≈ 0            λ₁ >> λ₂               λ₁ ≈ λ₂ >> 0                                                  .
```

| Situation | $\lambda_1$ | $\lambda_2$ | Bedeutung |
|-----------|-------------|-------------|-----------|
| **Flach** | klein | klein | Keine Änderung in beide Richtungen |
| **Kante** | groß | klein | Starke Änderung nur senkrecht zur Kante |
| **Ecke** | groß | groß | Starke Änderung in **beide** Richtungen |

Aber ... Harris suchte eine Lösung ohne Eigenwertberechnung! Harris nutzt einen Trick - Determinante und Spur kodieren die Eigenwerte:

$$
R = \det(M) - k \cdot \text{trace}(M)^2
$$

Mit $k \approx 0.04-0.06$

**Warum keine Eigenwertberechnung?**

1. Numerische Stabilität: Die Wurzelberechnung kann bei sehr kleinen oder sehr ähnlichen Eigenwerten zu numerischen Problemen führen (Division durch kleine Zahlen, Rundungsfehler)
2. Ausreichend für die Aufgabe: Harris braucht keine exakten Eigenwerte - er braucht nur eine Entscheidungsfunktion die sagt: "Ecke oder nicht". Die Kombination $\det(M) - k \cdot \text{trace}(M)^2$ liefert genau das, ohne die Eigenwerte explizit zu kennen.
3. Historisch (1988): Damals war die Rechenperformance tatsächlich noch ein relevanter Faktor.

> Der Harris-Detektor ist rotationsinvariant, weil Determinante und Spur bei Rotation der Matrix erhalten bleiben:

**Shi-Tomasi (1994) - "Good Features to Track"**

Verbesserte Version: Verwendet direkt die kleinere Eigenwerte

$$
R = \min(\lambda_1, \lambda_2)
$$

Wenn $R > \text{threshold}$: Guter Feature-Punkt

**Visualisierung:**

> Diese Corner-Detektoren werden in SLAM-Systemen für Feature-Tracking verwendet!


```python -loadImage.py 
import cv2
import numpy as np

import urllib.request

def load_image_from_url(url):
    resp = urllib.request.urlopen(url)
    image_data = resp.read()
    image_array = np.asarray(bytearray(image_data), dtype=np.uint8)
    image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    return image
```
```python corner_detection.py
import cv2
import numpy as np
from loadImage import load_image_from_url

image = load_image_from_url('https://r4r.informatik.tu-freiberg.de/content/images/size/w960/2022/08/karl_kegel_bau1.jpg')

# Graustufenkonvertierung (notwendig für Corner Detection)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Harris Corner Detector
harris_response = cv2.cornerHarris(np.float32(gray), blockSize=2, ksize=3, k=0.04)

# Graustufenbild in BGR konvertieren für farbige Markierungen
image_harris = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
image_harris[harris_response > 0.01 * harris_response.max()] = [0, 0, 255]

# Shi-Tomasi Corner Detector ("Good Features to Track")
corners = cv2.goodFeaturesToTrack(gray, maxCorners=100, qualityLevel=0.01, minDistance=10)

# Graustufenbild in BGR konvertieren für farbige Markierungen
image_shi_tomasi = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
if corners is not None:
    for corner in corners:
        x, y = corner.ravel()
        cv2.circle(image_shi_tomasi, (int(x), int(y)), 5, (0, 255, 0), -1)

cv2.imwrite('harris_corners.png', image_harris)
cv2.imwrite('shi_tomasi_corners.png', image_shi_tomasi)
```
@LIA.eval(`["loadImage.py", "main.py"]`, `none`, `python3 main.py`, `*`)

> Immer noch zu viel Rechenaufwand für den Einsatz auf mobilen Robotern? Dann schauen wir uns jetzt einen extrem schnellen Detektor an: FAST!

**FAST (Features from Accelerated Segment Test)**

1. Wähle einen Pixel $p$ mit Intensität $I_p$
2. Betrachte 16 Pixel auf einem Kreis (Radius 3) um $p$
3. Ist $p$ eine Ecke, wenn $n$ aufeinanderfolgende Pixel heller/dunkler sind
4. Typisch: $n = 12$, Threshold $t = 20$

![](https://docs.opencv.org/3.4/fast_speedtest.jpg "OpenCV FAST Example")

Jedem der 16 Pixel wird ein Label zugewiesen:

+ Heller ($I_{kreis} > I_p + t$)
+ Dunkler ($I_{kreis}< I_p - t$)
+ Ähnlich (weder noch)

Dann wird geprüft: Gibt es 12 zusammenhängende Pixel, die alle dasselbe Label (heller oder dunkler) haben? Falls ja → Ecke erkannt.

### Feature Descriptoren: Rotated BRIEF

> Nachdem wir nun Ecken detektieren können, brauchen wir eine Möglichkeit, diese Ecken zu beschreiben, damit wir sie in verschiedenen Bildern wiedererkennen können. Hierfür verwenden wir Deskriptoren.

**BRIEF (Binary Robust Independent Elementary Features)**

Beschreibt eine Ecke durch Binärvergleiche:

1. Wähle $n$ Pixelpaare zufällig um die Ecke
2. Vergleiche deren Intensitäten
3. Speichere Ergebnis als Bit-String

$$
\text{BRIEF}(p) = \sum_{1 \leq i \leq n} 2^{i-1} \cdot \tau(p; x_i, y_i)
$$

Wobei:
$$
\tau(p; x, y) = \begin{cases} 1 & \text{wenn } I(p_x) < I(p_y) \\ 0 & \text{sonst} \end{cases}
$$

Typisch: $n = 256$ → 256-Bit-Deskriptor

https://www.cs.ubc.ca/~lowe/525/papers/calonder_eccv10.pdf

> **Kernidee**: BRIEF vergleicht Intensitäten von Pixelpaaren und speichert das Ergebnis als Bit. Zwei Deskriptoren werden durch die Hamming-Distanz (Anzahl unterschiedlicher Bits) verglichen - sehr schnell durch XOR-Operation!

**Limitierung von BRIEF:** Nicht rotationsinvariant!

Das folgende Beispiel zeigt, wie BRIEF funktioniert: Für einen Feature-Punkt werden zufällige Pixelpaare verglichen und das Ergebnis als Bit-String gespeichert.

```python -loadImage.py
import cv2
import numpy as np
import urllib.request

def load_image_from_url(url):
    resp = urllib.request.urlopen(url)
    image_data = resp.read()
    image_array = np.asarray(bytearray(image_data), dtype=np.uint8)
    image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    return image
```
```python brief_demo.py
import cv2
import numpy as np
from loadImage import load_image_from_url

# Bild laden und in Graustufen konvertieren
image = load_image_from_url('https://r4r.informatik.tu-freiberg.de/content/images/size/w960/2022/08/karl_kegel_bau1.jpg')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
gray = cv2.GaussianBlur(gray, (5, 5), 0)  # Glättung für Stabilität

# Feature-Punkt (Zentrum für Demo)
feature_point = (gray.shape[1] // 2, gray.shape[0] // 2)
patch_size = 31  # Typische Patch-Größe für BRIEF

# Generiere zufällige Pixelpaare (vereinfachte BRIEF-Variante)
np.random.seed(42)  # Reproduzierbarkeit
n_pairs = 16  # Reduziert für Visualisierung (normal: 256)
pairs = np.random.randint(-patch_size//2, patch_size//2, size=(n_pairs, 4))

# Berechne BRIEF-Deskriptor
def compute_brief(img, point, pairs):
    """Berechne einen vereinfachten BRIEF-Deskriptor"""
    descriptor = []
    px, py = point

    for i, (dx1, dy1, dx2, dy2) in enumerate(pairs):
        # Koordinaten der beiden Pixel im Paar
        x1, y1 = px + dx1, py + dy1
        x2, y2 = px + dx2, py + dy2

        # Bounds-Check
        if (0 <= x1 < img.shape[1] and 0 <= y1 < img.shape[0] and
            0 <= x2 < img.shape[1] and 0 <= y2 < img.shape[0]):
            # Vergleich: I(p1) < I(p2) => 1, sonst 0
            bit = 1 if img[y1, x1] < img[y2, x2] else 0
            descriptor.append(bit)
        else:
            descriptor.append(0)

    return descriptor

# Berechne Deskriptor
descriptor = compute_brief(gray, feature_point, pairs)

# Visualisierung
vis_image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
px, py = feature_point

# Zeichne Feature-Punkt
cv2.circle(vis_image, (px, py), 8, (0, 255, 0), 2)

# Zeichne einige Pixelpaare
colors = [(255, 0, 0), (0, 0, 255)]  # Blau = 0, Rot = 1
for i, (dx1, dy1, dx2, dy2) in enumerate(pairs[:8]):  # Nur erste 8 für Übersichtlichkeit
    x1, y1 = px + dx1, py + dy1
    x2, y2 = px + dx2, py + dy2
    color = colors[descriptor[i]]
    cv2.line(vis_image, (x1, y1), (x2, y2), color, 1)
    cv2.circle(vis_image, (x1, y1), 3, color, -1)
    cv2.circle(vis_image, (x2, y2), 3, color, -1)

# Ergebnis ausgeben
print("BRIEF-Deskriptor Demonstration")
print("=" * 40)
print(f"Feature-Punkt: ({px}, {py})")
print(f"Anzahl Bit-Vergleiche: {n_pairs}")
print(f"\nBinärer Deskriptor: {''.join(map(str, descriptor))}")
print(f"Als Integer: {int(''.join(map(str, descriptor)), 2)}")
print(f"\nVergleich zweier Deskriptoren (Hamming-Distanz):")

# Demonstriere Hamming-Distanz
descriptor2 = compute_brief(gray, (px + 5, py + 5), pairs)  # Leicht verschoben
hamming = sum(a != b for a, b in zip(descriptor, descriptor2))
print(f"Deskriptor 1: {''.join(map(str, descriptor))}")
print(f"Deskriptor 2: {''.join(map(str, descriptor2))}")
print(f"Hamming-Distanz: {hamming} Bits unterschiedlich")

cv2.imwrite('brief_visualization.png', vis_image)
print("\nVisualisierung gespeichert: brief_visualization.png")
print("(Blaue Linien = Bit 0, Rote Linien = Bit 1)")
```
@LIA.eval(`["loadImage.py", "main.py"]`, `none`, `python3 main.py`, `*`)

**Gibt es auch eine rotationsinvariante Umsetzung?**

ORB (Oriented FAST and Rotated BRIEF) ist die rotationsinvariante Variante von BRIEF:


1. Orientierung berechnen: Für jeden Keypoint wird die dominante Orientierung mittels Intensity Centroid bestimmt:
$$\theta = \arctan2(m_{01}, m_{10})$$ wobei $m_{01}$ und $m_{10}$ die Bildmomente im Patch sind.

> **Was sind Bildmomente?** Die Bildmomente beschreiben den "Schwerpunkt der Intensität" in einem kreisförmigen Patch (typisch Radius $r = 15$) um den Keypoint:
>
> ```ascii
> FAST-Detektion:              Moment-Berechnung:
>
>      · · · · ·                  ░░░░░░░░░░░
>     ·         ·                ░░░░░░░░░░░░░
>    ·           ·              ░░░░░░░░░░░░░░░
>    ·     ●     ·      →       ░░░░░░░●░░░░░░░
>    ·           ·              ░░░░░░░░░░░░░░░
>     ·         ·                ░░░░░░░░░░░░░
>      · · · · ·                  ░░░░░░░░░░░
>
>    16 Pixel (r=3)             Voller Patch (r=15)
>    für Corner-Test            für Orientierung                                                            .
> ```
>
> **Analogie zur Mechanik:** Der Intensity Centroid entspricht dem Schwerpunkt eines Körpers, wobei Pixelintensitäten die Rolle der Massen übernehmen:
>
> | Mechanik | Bildverarbeitung |
> |----------|------------------|
> | Masse $m_i$ an Position $(x_i, y_i)$ | Intensität $I(x,y)$ an Pixel $(x,y)$ |
> | Gesamtmasse $M = \sum m_i$ | $m_{00} = \sum I(x,y)$ |
> | Schwerpunkt $\bar{x} = \frac{\sum m_i x_i}{M}$ | Centroid $C_x = \frac{m_{10}}{m_{00}}$ |

2. Rotierte Pixelpaare: Die vordefinierten BRIEF-Pixelpaare werden entsprechend der Orientierung $\theta$ rotiert, bevor der Deskriptor berechnet wird.
3. Berechnung des Deskriptors: Wie bei BRIEF, aber mit den rotierten Pixelpaaren.

**Weitere Deskriptoren (Float-basiert):**

Neben binären Deskriptoren wie ORB/BRIEF gibt es Float-Deskriptoren, die kontinuierliche Werte speichern:

<!-- data-type="none" -->
| Deskriptor | Dimension | Prinzip | Besonderheit |
|------------|-----------|---------|--------------|
| **SIFT** | 128 floats | Histogramme der Gradientenrichtungen in 4×4 Subregionen | Skalen- und rotationsinvariant, patentiert bis 2020 |
| **SURF** | 64 floats | Haar-Wavelet-Antworten in Subregionen | Schneller als SIFT, ähnliche Qualität |
| **AKAZE** | variabel | Nichtlineare Skalierungsräume mit Modified Local Difference Binary | Open-source Alternative zu SIFT |

> **Wann welchen Deskriptor?**
>
> - **ORB/BRIEF**: Echtzeitanwendungen (SLAM, Tracking) - schnell durch Hamming-Distanz
> - **SIFT/SURF**: Wenn Genauigkeit wichtiger als Geschwindigkeit ist (Panorama-Stitching, 3D-Rekonstruktion)
> - **AKAZE**: Guter Kompromiss - robust und frei verfügbar

### Feature Matching

    --{{0}}--
Nachdem wir Features in zwei Bildern gefunden haben, müssen wir korrespondierende Punkte finden.

**Brute-Force Matcher**

Einfachster Ansatz: Vergleiche jeden Deskriptor mit jedem anderen

+ Für binäre Deskriptoren (ORB, BRIEF): **Hamming-Distanz**
+ Für Float-Deskriptoren (SIFT, SURF): **Euklidische Distanz**

Hamming-Distanz = Anzahl unterschiedlicher Bits

```python
# Brute-Force Matcher für ORB (Hamming)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(descriptors1, descriptors2)

# Sortiere nach Distanz
matches = sorted(matches, key=lambda x: x.distance)
```

**Interaktives Beispiel: Ähnlichste Ecken mit ORB finden**

Das folgende Beispiel findet alle ORB-Features in einem Bild und sucht die ähnlichsten Ecken basierend auf der Hamming-Distanz ihrer Deskriptoren. Die Implementierung:

1. **Detektion**: ORB extrahiert bis zu 100 Keypoints mit FAST-Detektor
2. **Deskription**: Für jeden Keypoint wird ein 256-Bit BRIEF-Deskriptor berechnet (32 Bytes)
3. **Matching**: Alle Paare werden verglichen - die Hamming-Distanz zählt unterschiedliche Bits
4. **Visualisierung**: Die 10 ähnlichsten Paare (kleinste Distanz) werden grün markiert

> **Beobachtung:** Ähnliche Deskriptoren bedeuten ähnliche lokale Textur - nicht zwingend semantisch gleiche Objekte. Fensterecken links und rechts haben oft ähnliche Gradienten und werden daher als "ähnlich" erkannt.

```python -loadImage.py
import cv2
import numpy as np
import urllib.request

def load_image_from_url(url):
    resp = urllib.request.urlopen(url)
    image_data = resp.read()
    image_array = np.asarray(bytearray(image_data), dtype=np.uint8)
    image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    return image
```
```python orb_similar_corners.py
import cv2
import numpy as np
from loadImage import load_image_from_url

# Bild laden
image = load_image_from_url('https://r4r.informatik.tu-freiberg.de/content/images/size/w960/2022/08/karl_kegel_bau1.jpg')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# ORB-Detektor erstellen
orb = cv2.ORB_create(nfeatures=100)

# Keypoints und Deskriptoren berechnen
keypoints, descriptors = orb.detectAndCompute(gray, None)
print(f"Gefundene Keypoints: {len(keypoints)}")
print(f"Deskriptor-Shape: {descriptors.shape} (100 Keypoints x 32 Bytes = 256 Bits)")

# Alle Paare mit Hamming-Distanz berechnen
pairs = []
for i in range(len(descriptors)):
    for j in range(i + 1, len(descriptors)):
        distance = cv2.norm(descriptors[i], descriptors[j], cv2.NORM_HAMMING)
        pairs.append((distance, i, j))

# Nach Distanz sortieren und die 10 besten auswählen
pairs.sort(key=lambda x: x[0])
best_pairs = pairs[:10]

# Indizes der Keypoints in den besten Paaren sammeln
best_indices = set()
for dist, i, j in best_pairs:
    best_indices.add(i)
    best_indices.add(j)

print(f"\nDie 10 ähnlichsten Paare:")
for rank, (dist, i, j) in enumerate(best_pairs, 1):
    kp_i, kp_j = keypoints[i], keypoints[j]
    similarity = 100 * (1 - dist/256)
    print(f"  {rank}. KP {i} ({kp_i.pt[0]:.0f},{kp_i.pt[1]:.0f}) <-> "
          f"KP {j} ({kp_j.pt[0]:.0f},{kp_j.pt[1]:.0f}): "
          f"Distanz={int(dist)}, Ähnlichkeit={similarity:.1f}%")

# Visualisierung
vis_image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

# Alle Keypoints in Grau zeichnen
for i, kp in enumerate(keypoints):
    if i not in best_indices:
        cv2.circle(vis_image, (int(kp.pt[0]), int(kp.pt[1])), 4, (128, 128, 128), 1)

# Die Keypoints der 10 besten Paare in Grün hervorheben
for idx in best_indices:
    kp = keypoints[idx]
    cv2.circle(vis_image, (int(kp.pt[0]), int(kp.pt[1])), 8, (0, 255, 0), 2)

cv2.imwrite('orb_similar_corners.png', vis_image)
print("\nVisualisierung gespeichert: orb_similar_corners.png")
print("(Grüne Kreise = Keypoints der 10 ähnlichsten Paare)")
```
@LIA.eval(`["loadImage.py", "main.py"]`, `none`, `python3 main.py`, `*`)

> **Beobachtung:** Die ähnlichsten Ecken haben oft eine ähnliche lokale Struktur - z.B. zwei Fensterecken oder zwei Gebäudekanten mit ähnlicher Textur.

**RANSAC für robuste Geometrie**

RANSAC folgt auf das Deskriptor-Matching und wirkt als geometrischer Konsistenzfilter auf die Matches.

Entferne Outliers durch geometrische Konsistenz auf der Basis von Essenzial- oder Homographiemodellen:

1. Wähle zufällig minimale Menge von Matches
2. Berechne Homographie $H$ (oder Fundamentalmatrix $F$)
3. Zähle Inliers (Matches konsistent mit $H$)
4. Wiederhole und wähle beste Lösung

```python
# RANSAC für Homographie
H, mask = cv2.findHomography(
    src_pts, dst_pts,
    cv2.RANSAC,
    ransacReprojThreshold=5.0
)

# mask[i] == 1: Inlier, mask[i] == 0: Outlier
inliers = src_pts[mask.ravel() == 1]
```

> Wir varieren die Modellparameter und suchen nach der besten Lösung mit den meisten Inliers! RANSAC arbeitet nicht auf den Des

!?[](https://www.youtube.com/watch?v=EwlKwbyK8GI)

### Haar Features und Gesichtserkennung

    --{{0}}--
Ein weiterer klassischer Algorithmus für Objekterkennung sind Haar-artige Features, die durch Viola und Jones 2001 bekannt wurden. Diese Methode war bahnbrechend für die Echtzeit-Gesichtserkennung und wird in OpenCV als Haar Cascade Classifier implementiert.

**Grundidee der Haar Features:**

Haar Features basieren auf der Berechnung von Helligkeitsunterschieden zwischen benachbarten Rechteckbereichen im Bild:

```ascii
Haar Feature Typen:

  Edge Features:          Line Features:         Four-Rectangle:
  ┌─────┬─────┐          ┌─────┬─────┬─────┐    ┌─────┬─────┐
  │█████│     │          │█████│     │█████│    │█████│     │
  │█████│     │          │█████│     │█████│    ├─────┼─────┤
  └─────┴─────┘          └─────┴─────┴─────┘    │     │█████│
                                                └─────┴─────┘
  Vertikal   Horizontal

  Feature-Wert = Σ(weiße Pixel) - Σ(schwarze Pixel)                                                      .
```

Diese einfachen Features können komplexe Strukturen erfassen:

+ **Kantenfeatures**: Erkennen Übergänge zwischen hellen und dunklen Bereichen
+ **Linienfeatures**: Erkennen dunkle Linien auf hellem Hintergrund (z.B. Augenbrauen)
+ **Rechteckfeatures**: Erkennen kontrastierende Bereiche (z.B. Nase heller als Augenpartie)

Parameter pro Feature in einer Stage sind z.B.:

+ Position (x, y) im Bildfenster
+ Größe (Breite, Höhe) der Rechtecke
+ Art des Features (Edge horizontal, Edge vertikal, Line etc.)
+ Gewicht (wie wichtig dieses Feature ist)
+ Schwellenwerte (Thresholds) zur Entscheidungsfindung

**Integralbilder für schnelle Berechnung:**

Der Trick für Echtzeit-Performance liegt in der Verwendung von Integralbildern:

$$II(x,y) = \sum_{x' \leq x, y' \leq y} I(x', y')$$

Mit dem Integralbild kann die Summe jedes Rechtecks in **konstanter Zeit O(1)** berechnet werden:

```ascii
Rechteck-Summe mit 4 Array-Zugriffen:

     A ────────── B
     │           │
     │  Rechteck │     Summe = II(D) - II(B) - II(C) + II(A)
     │           │
     C ────────── D                                                                                     .
```

**AdaBoost Cascade Classifier:**

Das Training verwendet AdaBoost, um aus tausenden möglicher Haar Features die relevantesten auszuwählen und in einer Kaskade anzuordnen:

```ascii
Cascade Structure:

  Bild-       Stage 1        Stage 2        Stage 3        Face
  Region  ─── (wenige    ─── (mehr     ─── (viele    ─── Detected!
               Features)      Features)      Features)

     │            │              │              │
     ▼            ▼              ▼              ▼
  Reject      Reject         Reject         Reject
  (schnell)   (schnell)      (langsam)      (langsam)                                                   .
```

+ Frühe Stufen haben wenige Features → schnelle Ablehnung von Nicht-Gesichtern
+ Spätere Stufen werden nur für vielversprechende Kandidaten ausgeführt
+ ~95% der Bildregionen werden bereits in den ersten Stufen verworfen

**OpenCV Implementierung:**

```python
import cv2

# Lade vortrainierten Haar Cascade Classifier
face_cascade = cv2.CascadeClassifier(
    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
)

# Bild laden und in Graustufen konvertieren
img = cv2.imread('gruppe.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Gesichtserkennung durchführen
faces = face_cascade.detectMultiScale(
    gray,
    scaleFactor=1.1,    # Skalierungsfaktor zwischen Scans
    minNeighbors=5,     # Mindestanzahl benachbarter Detektionen
    minSize=(30, 30)    # Minimale Gesichtsgröße
)

# Ergebnisse visualisieren
for (x, y, w, h) in faces:
    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

print(f"Gefundene Gesichter: {len(faces)}")
cv2.imwrite('detected_faces.jpg', img)
```

**Parameter erklärt:**

| Parameter | Bedeutung | Typischer Wert |
|-----------|-----------|----------------|
| `scaleFactor` | Verkleinerungsfaktor pro Scan-Durchlauf | 1.05 - 1.3 |
| `minNeighbors` | Erforderliche Überlappungen für Detektion | 3 - 6 |
| `minSize` | Minimale Objektgröße in Pixeln | (30, 30) |
| `maxSize` | Maximale Objektgröße in Pixeln | unbegrenzt |

**Weitere verfügbare Cascade Classifier in OpenCV:**

+ `haarcascade_eye.xml` - Augenerkennung
+ `haarcascade_smile.xml` - Lächeln-Erkennung
+ `haarcascade_profileface.xml` - Gesichter im Profil
+ `haarcascade_fullbody.xml` - Ganzkörper-Erkennung
+ `haarcascade_upperbody.xml` - Oberkörper

!?[](https://www.youtube.com/watch?v=hPCTwxF0qf4&t=103s)

## Deep Learning für Objekterkennung

> Deep Learning ist eine Kategorie des maschinellen Lernens, der auf tiefen künstlichen neuronalen Netzen basiert. „Deep“ bezeichnet die Anzahl hintereinandergeschalteter Verarbeitungsebenen (Layer) in einem neuronalen Netz.

+ Viele hintereinandergeschaltete Schichten
+ Automatische Merkmalsextraktion
+ Hoher Daten- und Rechenbedarf
+ Besonders erfolgreich bei unstrukturierten Daten

```ascii
Künstliche Intelligenz
 └─ Maschinelles Lernen
     └─ Deep Learning                                                   .
```

> „‚Deep‘ bezeichnet die Tiefe der Repräsentationshierarchie: Viele aufeinanderfolgende nichtlineare Transformationen, die aus Rohdaten schrittweise abstrakte Merkmale formen.“

| Aspekt          | Haar Cascade                      | Deep Learning (YOLO)    |
| --------------- | --------------------------------- | ----------------------- |
| Geschwindigkeit | Sehr schnell (CPU)                | Schnell (benötigt GPU)  |
| Genauigkeit     | Gut für frontale Gesichter        | Besser bei Variationen  |
| Robustheit      | Empfindlich auf Rotation          | Robust                  |
| Training        | Aufwendig, benötigt viele Samples | End-to-End Training     |
| Speicherbedarf  | Sehr gering (`~1 MB`)             | Größer (`~10-100 MB`)     |
| Anwendungsfall  | Embedded Systems, Echtzeit        | Server, komplexe Szenen |

> **Fazit:** Haar Cascade Classifier sind ein gutes Beispiel dafür, wie mit cleveren mathematischen Tricks (Integralbilder) und Machine Learning (AdaBoost) effiziente Objekterkennung möglich ist. Für viele Embedded-Anwendungen sind sie aufgrund ihrer Geschwindigkeit und geringen Ressourcenanforderungen immer noch relevant.

### DL Architekturen

> CNNs sind nicht das einzige Werkzeug des Deep Learning!

| Typ                                     | Typische Anwendungen                 | Beispiele für Anwendungen                                                                                                             |
| --------------------------------------- | ------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------- |
| **CNN (Convolutional Neural Networks)** | Bilder, Videos, räumliche Daten      | Objekterkennung (z.B. YOLO, Gesichtserkennung), Bildklassifikation (z.B. Katzen vs. Hunde), medizinische Bildanalyse (Tumorerkennung) |
| **RNN (Recurrent Neural Networks)**     | Zeitreihen, Sprache, Sequenzen       | Handschriftenerkennung, maschinelle Übersetzung, Sprachgenerierung, Aktienkursvorhersage                                              |
| **Transformers**                        | Sprache (NLP), Bild, multimodal      | Chatbots (z.B. ChatGPT), maschinelle Übersetzung, Textzusammenfassung, Bildunterschriften-Generierung                                 |
| **Fully Connected / Dense Nets**        | Klassifikation, einfache ML-Aufgaben | Kreditwürdigkeitsprüfung, Spam-Erkennung, Iris-Blumenklassifikation (ein klassisches ML-Beispiel)                                     |
| **Autoencoder, GANs, etc.**             | Datenkompression, Generierung        | Bildrauschen entfernen, Bilderzeugung (DeepFakes, Stiltransfer), Anomalieerkennung in Produktionsdaten                                |

### CNN-Grundlagen (Kurzer Einstieg)

**Convolutional Neural Network (CNN):**

```ascii
Input Image          Conv Layer        Pooling         FC Layer      Output
  ┌─────┐           ┌─────┐          ┌───┐          ┌─────┐       ┌─────┐
  │     │  ──────>  │     │  ──────> │   │ ──────>  │     │ ───>  │Cat  │
  │ 🐱  │  Filter   │ ▓▓▓ │  MaxPool │ ▓ │ Flatten  │█████│       │Dog  │
  │     │           │ ▓▓▓ │          │ ▓ │          │█████│       │Bird │
  └─────┘           └─────┘          └───┘          └─────┘       └─────┘
  224×224×3         112×112×64       56×56×64       4096          Classes                                  .
```

**Input Image (Eingabebild):**
Ein Farbbild mit z.B. **224×224 Pixeln** und **3 Farbkanälen** (Rot, Grün, Blau).
Das Bild wird als 3D-Array betrachtet: Höhe × Breite × Farbkanäle.

**Convolutional Layer (Faltungsschicht):**
Viele kleine Filter (z.B. 3×3) werden über das Bild geschoben.

* Jeder Filter erkennt lokale Merkmale wie Kanten, Ecken, Farben oder Texturen.
* Filterwerte werden **während des Trainings gelernt** und sind damit flexibel.
* Das Ergebnis sind mehrere **Feature Maps** (hier z.B. 64), die anzeigen, wo im Bild bestimmte Muster auftreten.

*Analogie:* Eine Schablone, die verschiedene Muster über das Bild legt und prüft, wo sie am besten passen.

**Pooling Layer (Pooling-Schicht):**
Reduziert die räumlichen Dimensionen der Feature Maps (hier von 112×112 auf 56×56).

* Typisch ist **Max-Pooling**, bei dem in kleinen Bereichen (z.B. 2×2) der größte Wert übernommen wird.
* Dadurch bleibt die wichtigste Information erhalten, während die Darstellung kompakter wird.
* Pooling macht das Modell robuster gegenüber kleinen Verschiebungen und reduziert Rechenaufwand.

*Analogie:* Eine Zusammenfassung oder Vergröberung, die das Bild kompakter und übersichtlicher macht.

**ReLU Activation (Aktivierungsfunktion) zwischen den Schichten:**
Führt eine einfache nichtlineare Transformation durch: $f(x) = \max(0, x)$

Diese Nichtlinearität ermöglicht es dem Netzwerk, komplexe Muster zu lernen.

*Hinweis:* ReLU ist meist **kein eigener Layer**, sondern wird direkt nach jeder Conv- oder FC-Schicht angewendet.

**Fully Connected Layer (Vollständig verbundene Schicht):**
Am Ende des Netzes werden alle extrahierten Merkmale **flachgedrückt (flattened)** und als Vektor in den FC-Layer eingespeist.

* Jeder Eingang ist mit jedem Neuron verbunden.
* Der FC-Layer kombiniert alle Merkmale und entscheidet, zu welcher Klasse (z.B. Katze, Hund, Vogel) das Bild gehört.

> Was bedeutet „Deep“ genau in deinem CNN-Diagramm?

„Deep“ bedeutet, dass viele Convolutional Layers (plus weitere Schichten wie Pooling und Aktivierung) hintereinander geschaltet werden, um eine Hierarchie von Merkmalen zu lernen — von einfachen Kanten bis zu komplexen Objekten.
Diese Tiefe ermöglicht es dem Netzwerk, sehr komplexe Muster und Zusammenhänge in den Daten zu erfassen.

### YOLO: You Only Look Once

YOLO ist ein **Single-Shot Detector**: Das gesamte Bild wird in einem einzigen Durchgang verarbeitet, und alle Objekte werden gleichzeitig erkannt. Das macht YOLO extrem schnell!

**YOLO-Prinzip:**

1. **Teile das Bild in ein Grid** (z.B. 13×13 Zellen)
2. **Jede Grid-Zelle sagt vorher:**

   * $B$ Bounding Boxes mit Koordinaten ((x, y, w, h)) und Confidence-Wert
   * $C$ Klassenwahrscheinlichkeiten
3. **Non-Maximum Suppression (NMS)** entfernt mehrfach erkannte Objekte (Überlappungen)

```ascii
Grid-basierte Objekterkennung:

  ┌─┬─┬─┬─┬─┬─┬─┐
  ├─┼─┼─┼─┼─┼─┼─┤        Grid-Zelle (3,2) detektiert:
  ├─┼─┼█┼─┼─┼─┼─┤        • Bounding Box: (x=3.2, y=2.7, w=2.1, h=3.5)
  ├─┼─┼█┼─┼─┼─┼─┤  ───>  • Confidence: 0.89
  ├─┼─┼█┼─┼─┼─┼─┤        • Klasse: "person"
  ├─┼─┼─┼─┼─┼─┼─┤
  └─┴─┴─┴─┴─┴─┴─┘
   7×7 Grid                                                                                               .
```

**Evolution der YOLO-Familie (Auswahl):**

| Version | Jahr | mAP (COCO) | FPS | Besonderheiten                  |
| ------- | ---- | ---------- | --- | ------------------------------- |
| YOLOv1  | 2016 | ~63.4%     | 45  | Erste Single-Shot Detection     |
| YOLOv2  | 2017 | ~78.6%     | 67  | Batch Norm, Anchor Boxes        |
| YOLOv3  | 2018 | ~57.9%     | 45  | Multi-Scale Predictions         |
| YOLOv4  | 2020 | ~65.7%     | 65  | CSPDarknet Backbone             |
| YOLOv5  | 2020 | ~67.3%     | 140 | PyTorch Implementation, einfach |
| YOLOv8  | 2023 | ~70.0%     | 120 | Aktuell empfohlen               |

> **Für Übung 2 verwenden wir YOLOv8** — beste Balance zwischen Performance und Benutzerfreundlichkeit!


### YOLOv8 im Detail

YOLOv8 ist die aktuell empfohlene Version von Ultralytics. Sie bietet eine sehr einfache Python-API und eignet sich gut für Integration in ROS 2 und andere Anwendungen.

**Architektur-Überblick:**

```ascii
YOLOv8 Architektur:

Input               Backbone           Neck              Head
640×640×3          (CSPDarknet)     (PAN-FPN)      (Detection Head)
┌─────┐             ┌─────┐          ┌─────┐          ┌─────┐
│     │  ────────>  │ ▓▓▓ │ ──────>  │ ▓▓  │ ──────>  │BBox │
│ 🚶  │  Feature     │ ▓▓▓ │  Multi-  │ ▓▓  │  Predict │Class│
│     │  Extraction │ ▓▓▓ │  Scale   │ ▓▓  │          │Conf │
└─────┘             └─────┘          └─────┘          └─────┘
                    ca. 20 Mio        Fusion          Outputs  
                    Parameter         Layers                                                               .
```

**YOLOv8 Modell-Varianten:**

| Modell  | Parameteranzahl | mAP  | Speed (ms) | Typische Verwendung            |
| ------- | --------------- | ---- | ---------- | ------------------------------ |
| YOLOv8n | 3.2 Mio         | 37.3 | 1.2        | Embedded, Edge-Devices         |
| YOLOv8s | 11.2 Mio        | 44.9 | 1.9        | Gute Balance, schneller Laptop |
| YOLOv8m | 25.9 Mio        | 50.2 | 3.2        | Standardwahl                   |
| YOLOv8l | 43.7 Mio        | 52.9 | 4.5        | Höhere Genauigkeit             |
| YOLOv8x | 68.2 Mio        | 53.9 | 6.8        | Beste Genauigkeit              |

> **Empfehlung für Übung 2:** YOLOv8n oder YOLOv8s — schnell genug für Echtzeit auf Laptops!

!?[](https://www.youtube.com/watch?v=svn9-xV7wjk)

### YOLOv8 im Detail

YOLOv8 ist die aktuell empfohlene Version von Ultralytics. Sie bietet eine sehr einfache Python-API und ist optimal für ROS 2 Integration geeignet.

**Installation und Nutzung:**

```python
# Installation
pip install ultralytics

# Import
from ultralytics import YOLO

# Vortrainiertes Modell laden (COCO-Dataset)
model = YOLO('yolov8n.pt')  # nano model

# Inferenz auf einem Bild
results = model('image.jpg')

# Oder auf Video/Webcam
results = model('video.mp4', stream=True)
```

**Output-Format verstehen:**

```python
# Ergebnisse iterieren
for result in results:
    boxes = result.boxes  # Bounding Boxes

    for box in boxes:
        # Koordinaten
        x1, y1, x2, y2 = box.xyxy[0]  # [x_min, y_min, x_max, y_max]

        # Oder: Center + Width/Height
        x_center, y_center, w, h = box.xywh[0]

        # Confidence Score
        confidence = box.conf[0]

        # Klassen-ID (COCO)
        class_id = int(box.cls[0])

        # Klassen-Name
        class_name = model.names[class_id]

        print(f"{class_name}: {confidence:.2f} at ({x1}, {y1})")
```

### COCO-Dataset und Personendetektion

    --{{0}}--
YOLOv8 wird standardmäßig auf dem COCO-Dataset trainiert. Dieses enthält 80 Objektklassen.

**COCO (Common Objects in Context):**

+ 80 Objektklassen
+ 330K Bilder
+ 1.5M Objektinstanzen
+ "Industry-Standard" für Object Detection

https://cocodataset.org/#home

**Wichtige Klassen (Auswahl):**

| Class ID | Name       | Class ID | Name       |
| -------- | ---------- | -------- | ---------- |
| 0        | **person** | 5        | bus        |
| 1        | bicycle    | 16       | dog        |
| 2        | car        | 17       | cat        |
| 3        | motorcycle | 62       | chair      |
| 4        | airplane   | 73       | book       |

**Personendetektion mit YOLOv8:**

```python
from ultralytics import YOLO

# Modell laden
model = YOLO('yolov8n.pt')

# Bild laden
image = cv2.imread('scene.jpg')

# Inferenz
results = model(image)

# Nur Personen filtern
persons = []
for box in results[0].boxes:
    if int(box.cls[0]) == 0:  # person class
        if box.conf[0] > 0.5:  # confidence threshold
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            persons.append({
                'bbox': [x1, y1, x2, y2],
                'confidence': float(box.conf[0]),
                'center': ((x1+x2)/2, (y1+y2)/2)  # Wichtig für Stereo!
            })

print(f"Detected {len(persons)} persons")
```

> Experimentieren Sie mit der Demo Implementierung im Ordner /yolo_Example.

### Von Bounding Box zu 3D-Position

    --{{0}}--
Dies ist der entscheidende Schritt für Übung 2: Wir kombinieren YOLOv8-Detektionen mit der Disparitätskarte aus der Stereo-Kamera!

**Pipeline:**

```ascii
Step 1: Detection        Step 2: Center Point      Step 3: Depth Lookup
┌─────────┐              ┌─────────┐              ┌─────────┐
│  ┌───┐  │              │    ●    │              │    ●    │
│  │ 🚶│  │  ───────────>│  (cx,cy)│  ──────────> │  Z=3.2m │
│  └───┘  │  YOLOv8      │         │  Disparity   │         │
└─────────┘              └─────────┘  Map         └─────────┘

Step 4: 3D Coordinates
   (X, Y, Z) in camera frame
        │
        v
   Transform to robot frame (TF2)                                                                         .
```

**Mathematik:**

> Aus L06 wissen wir: Die Tiefe (Entfernung) eines Punkts zur Kamera ist proportional zum Produkt aus Brennweite und Abstand zwischen den Kameras und umgekehrt proportional zur Disparität (dem Pixelversatz) dieses Punkts in den Stereo-Bildern.

$$
Z = \frac{f \cdot b}{d}
$$

Wobei:

+ $f$ ist die Brennweite der Kamera (in Pixeln).
+ $b$ ist die Baseline, also der Abstand zwischen den beiden Kameras.
+ $d$ ist die Disparität, also die Differenz der Position des Punkts in den zwei Bildern.

Nun nutzt du $Z$ und die Bildkoordinaten $(u, v)$, um die reale Position im Kamerakoordinatensystem zu berechnen:

$$
X = \frac{(u - c_x) \cdot Z}{f_x}
$$

$$
Y = \frac{(v - c_y) \cdot Z}{f_y}
$$

mit 

+ $c_x, c_y$ sind die Koordinaten des Hauptpunkts (Principal Point) in Pixeln
+ $f_x, f_y$ sind die Brennweiten in Pixeln entlang der $x$- bzw. $y$-Achse (aus der Kamerakalibrierung)

**Python-Implementierung:**

```python
def get_3d_position(bbox, disparity_map, camera_info):
    """
    Berechne 3D-Position aus Bounding Box und Disparitätskarte

    Args:
        bbox: [x1, y1, x2, y2] in Pixel
        disparity_map: Disparitätskarte (gleiche Auflösung wie Bild)
        camera_info: Kamera-Kalibrierung (fx, fy, cx, cy, baseline)

    Returns:
        (X, Y, Z) in Metern (camera frame)
    """
    # Center der Bounding Box
    cx = int((bbox[0] + bbox[2]) / 2)
    cy = int((bbox[1] + bbox[3]) / 2)

    # Disparität an diesem Punkt
    disparity = disparity_map[cy, cx]

    # Tiefe berechnen (aus L06)
    if disparity > 0:
        Z = (camera_info['fx'] * camera_info['baseline']) / disparity
    else:
        return None  # Keine gültige Tiefe

    # 3D-Koordinaten
    X = (cx - camera_info['cx']) * Z / camera_info['fx']
    Y = (cy - camera_info['cy']) * Z / camera_info['fy']

    return (X, Y, Z)
```

**Robustheit verbessern:**

Problem: Disparität kann Rauschen enthalten oder ungültig sein

Lösungen:

```python
# 1. Verwende Median über mehrere Pixel
window_size = 5
cx_min = max(0, cx - window_size//2)
cx_max = min(disparity_map.shape[1], cx + window_size//2)
cy_min = max(0, cy - window_size//2)
cy_max = min(disparity_map.shape[0], cy + window_size//2)

window = disparity_map[cy_min:cy_max, cx_min:cx_max]
disparity = np.median(window[window > 0])  # Ignoriere 0-Werte

# 2. Plausibilitäts-Check
if 0.5 < Z < 10.0:  # Personen zwischen 0.5m und 10m
    return (X, Y, Z)
else:
    return None
```

## Object Tracking

    --{{0}}--
Objekterkennung liefert uns Detektionen in einzelnen Bildern. Tracking verbindet diese über Zeit und ermöglicht uns, Bewegungen vorherzusagen und IDs zu erhalten.

**Vorteile gegenüber reiner Detektion:**

+ **Temporal Coherence**: Glättung über Zeit reduziert Rauschen
+ **ID-Konsistenz**: "Person #1" bleibt "Person #1" über mehrere Frames
+ **Prädiktive Suche**: Effizienter durch Einschränkung des Suchbereichs
+ **Okklusion-Handling**: Objekte können kurzzeitig verdeckt werden
+ **Trajektorien**: Geschwindigkeit und Richtung bestimmen

**Anwendungen:**

+ Autonome Fahrzeuge: Fußgänger-Trajektorien vorhersagen
+ Person-Following Roboter: Verfolge spezifische Person
+ Videoanalyse: Zähle Personen, die durch Tür gehen

### Optical Flow: Lucas-Kanade

    --{{0}}--
Optischer Fluss beschreibt die Bewegung von Pixeln zwischen zwei aufeinanderfolgenden Bildern.

**Annahmen:**

1. **Brightness Constancy**: Pixel-Intensität bleibt konstant
   $$I(x, y, t) = I(x + \Delta x, y + \Delta y, t + \Delta t)$$

2. **Small Motion**: Bewegung ist klein zwischen Frames

3. **Spatial Coherence**: Nachbar-Pixel bewegen sich ähnlich

**Grundannahme der Lucas-Kanade Methode:** Ein Pixel behält seine Helligkeit zwischen den Frames: $I(x, y, t) = I(x + u, y + v, t + 1)$

**Realisierung:**

| Gradient | Berechnung | Bedeutung |
|----------|------------|-----------|
| $I_x$ | Sobel in x-Richtung | Helligkeitsänderung horizontal |
| $I_y$ | Sobel in y-Richtung | Helligkeitsänderung vertikal |
| $I_t$ | Frame2 - Frame1 | Helligkeitsänderung zeitlich |

Die Verschiebungen werden auf Basis der **Helligkeitsgradienten** berechnet - nicht durch Deskriptor-Matching wie bei ORB/SIFT.

```ascii 
Fenster um Feature-Punkt (z.B. 5×5 Pixel):

Frame t:              Frame t+1:           Differenz (I_t):
┌─────────────┐       ┌─────────────┐      ┌─────────────┐
│ 120 125 130 │       │ 118 123 128 │      │ -2  -2  -2  │
│ 140 200 145 │  →    │ 142 202 147 │  =   │ +2  +2  +2  │
│ 135 130 125 │       │ 137 132 127 │      │ +2  +2  +2  │
└─────────────┘       └─────────────┘      └─────────────┘                                                  .
``` 

Aus den räumlichen Gradienten ($I_x$, $I_y$) und der zeitlichen Änderung ($I_t$) wird ein **überbestimmtes Gleichungssystem** aufgestellt (eine Gleichung pro Pixel im Fenster):

**Beispiel für 2 Pixel:**

Pixel 1 an Position $(x_1, y_1)$: $\quad I_{x,1} \cdot u + I_{y,1} \cdot v = -I_{t,1}$

Pixel 2 an Position $(x_2, y_2)$: $\quad I_{x,2} \cdot u + I_{y,2} \cdot v = -I_{t,2}$

In Matrixform:

$$
\begin{bmatrix} I_{x,1} & I_{y,1} \\ I_{x,2} & I_{y,2} \end{bmatrix}
\begin{bmatrix} u \\ v \end{bmatrix} =
\begin{bmatrix} -I_{t,1} \\ -I_{t,2} \end{bmatrix}
$$

Bei einem 5×5 Fenster haben wir **25 Gleichungen** für nur **2 Unbekannte** $(u, v)$ → überbestimmt! Die Lösung erfolgt per **Least Squares**.

**Unterschied zu Feature-Matching:**

| Lucas-Kanade | ORB/SIFT Matching |
|--------------|-------------------|
| Nutzt **Gradienten** (Helligkeitsänderungen) | Nutzt **Deskriptoren** (Bit-Strings) |
| Sucht minimalen Helligkeitsfehler | Sucht minimale Hamming-Distanz |
| Braucht **kleine Bewegungen** | Funktioniert bei großen Verschiebungen |
| Sehr schnell (nur Matrixinversion) | Aufwendiger (alle Paare vergleichen) |

> Die Verschiebung wird also **nicht durch Suche nach ähnlichen Mustern** gefunden, sondern durch **Lösen einer Gleichung**, die beschreibt, wohin sich der Helligkeitsgradient bewegt haben muss.

### DeepSORT: Deep Simple Online Realtime Tracking

    --{{0}}--
DeepSORT ist der aktuelle Standard für Multi-Object Tracking. Er kombiniert Kalman-Filter mit Deep Learning Features.

**DeepSORT-Pipeline:**

```ascii
Frame t-1                     Frame t
┌─────────┐                  ┌─────────┐
│ Person1 │                  │ Person? │
│ Person2 │  ──────────────> │ Person? │
│ Person3 │  Predict & Match │ Person? │
└─────────┘                  └─────────┘
    │                             │
    v                             v
Kalman Filter Prediction    YOLO Detections
    │                             │
    └──────────┬──────────────────┘
               v
        Hungarian Algorithm
        (Data Association)
               │
               v
        ┌─────────┐
        │ Track 1 │ ← Person1 matched
        │ Track 2 │ ← Person2 matched
        │ Track 3 │ ← Person3 lost (tentative)
        │ Track 4 │ ← New detection
        └─────────┘                                                                                    .
```

**Komponenten:**

**1. Detection (YOLO)**
   - Liefert Bounding Boxes pro Frame

**2. Kalman Filter**
   - Prädiziert Position im nächsten Frame
   - State: $(x, y, a, h, \dot{x}, \dot{y}, \dot{a}, \dot{h})$
     - $(x, y)$ = Center
     - $a$ = Aspect Ratio
     - $h$ = Höhe
     - Ableitungen = Geschwindigkeiten

**3. Appearance Descriptor**
   - CNN-Features für jede Detection
   - Hilft bei Re-Identification nach Okklusion
   - Typisch: 128-dim Feature Vector

**4. Hungarian Algorithm**
   - Optimale Zuordnung: Detection → Track
   - Minimiert kombinierte Kostenfunktion:
     $$c_{i,j} = \lambda \cdot d_{\text{Mahalanobis}} + (1-\lambda) \cdot d_{\text{Cosine}}$$

**5. Track Management**
   - **Confirmed**: Track existiert über $n$ Frames
   - **Tentative**: Neue Detection, noch unsicher
   - **Deleted**: Track verloren über $m$ Frames

**Kostenfunktion im Detail:**

$$
d_{\text{Mahalanobis}} = \sqrt{(d_j - y_i)^T S_i^{-1} (d_j - y_i)}
$$

+ $d_j$ = Detection $j$
+ $y_i$ = Kalman-Prädiktion für Track $i$
+ $S_i$ = Kovarianzmatrix

$$
d_{\text{Cosine}} = 1 - \frac{r_j^T \cdot r_i^k}{||r_j|| \cdot ||r_i^k||}
$$

+ $r_j$ = Appearance Feature von Detection $j$
+ $r_i^k$ = Appearance Features von Track $i$ (letzten $k$ Frames)


## 3D-Objekterkennung in Punktwolken

Bisher haben wir 2D-Bilder betrachtet. Für mobile Roboter sind Punktwolken aus Lidar oder Stereo-Kameras ebenso wichtig. Hier lernen wir grundlegende 3D-Algorithmen.

### RANSAC: Plane Segmentation

    --{{0}}--
RANSAC (Random Sample Consensus) ist ein robuster Algorithmus zum Fitten von Modellen in verrauschten Daten.

**Problem**: Finde Boden-Ebene in Punktwolke

**Ebenen-Gleichung:**

$$
ax + by + cz + d = 0
$$

Normalisiert mit $\sqrt{a^2 + b^2 + c^2} = 1$

**RANSAC-Algorithmus:**

Für k Iterationen:

  1. Wähle zufällig 3 Punkte
  2. Berechne Ebene durch diese Punkte
  3. Zähle Inliers (Punkte mit Distanz < threshold)
  4. Speichere beste Lösung

Rückgabe: Ebene mit meisten Inliers 

RANSAC ist keine klassische kontinuierliche Optimierung (z.B. Gradient-Descent auf einer Differenzierbaren Kostenfunktion), sondern:

+ Stochastisch: Es probiert viele zufällige Modell-Kandidaten aus.
+ Diskret: Bewertet Modelle nach Anzahl der Inlier (eine diskrete, nicht differenzierbare "Kostenfunktion").

**Mathematik:**

Ebene durch 3 Punkte $P_1, P_2, P_3$:

Normalenvektor:
$$
\vec{n} = (P_2 - P_1) \times (P_3 - P_1)
$$

Ebenengleichung:
$$
\vec{n} \cdot (P - P_1) = 0
$$

Distanz Punkt $P$ zur Ebene:
$$
d = \frac{|ax_P + by_P + cz_P + d|}{\sqrt{a^2 + b^2 + c^2}}
$$

Oft wird das beste Modell noch durch eine Least-Squares-Optimierung auf den gefundenen Inliern feinjustiert — das ist dann eine klassische kontinuierliche Optimierung, die Fehler quadratisch minimiert.

**Anwendungen:**

+ Boden-Entfernung für Navigation
+ Wand-Detektion
+ Tisch-Oberflächen finden (für Grasping)

### Euclidean Clustering

    --{{0}}--
Nach dem Entfernen des Bodens wollen wir einzelne Objekte separieren - das macht Euclidean Clustering.

**Prinzip:**

```ascii
Punktwolke nach Boden-Entfernung:

  ··  ··    ···         Cluster 1  Cluster 2  Cluster 3
  ··  ··    ···    →      ██         ██         ███
 ────────────────       ════════════════════════════
  (Boden entfernt)              (Boden)                                                                 .
```

**Algorithmus (DBSCAN-ähnlich):**

```
1. Erstelle KD-Tree für schnelle Nachbarsuche
2. Für jeden Punkt p:
   - Wenn p schon zugeordnet: Skip
   - Erstelle neues Cluster
   - Finde alle Nachbarn in Radius r
   - Füge Nachbarn rekursiv hinzu
3. Filtere Cluster nach Größe (min/max Punkte)
```

### 3D Feature Descriptors: FPFH

    --{{0}}--
Um Objekte in 3D zu erkennen, brauchen wir Deskriptoren - ähnlich wie ORB in 2D.

**FPFH (Fast Point Feature Histograms)**

Beschreibt die lokale Geometrie um einen Punkt herum.

**Berechnung:**

```ascii
Schritt 1: Normalen          Schritt 2: SPFH             Schritt 3: FPFH
                             (für Punkt p)               (Aggregation)

    ↗ n₁                         p                           p
   ·                            /|\                         /|\
  ↗ n₂                         / | \                    ───/─|─\───
 ·    ·                      q₁  q₂  q₃                  gewichtete
    ↗ n₃                     Winkel α,φ,θ               Summe der
   ·                         → Histogramm                Nachbar-SPFHs                                      .
```

**Schritt 1: Surface Normals berechnen**

Für jeden Punkt wird die lokale Oberflächenorientierung geschätzt. Dazu wird eine Ebene durch die Nachbarpunkte gefittet (PCA oder Least Squares) - der Normalenvektor dieser Ebene ist die Surface Normal.

**Schritt 2: SPFH (Simplified Point Feature Histogram)**

Für einen Punkt $p$ betrachten wir **alle** Nachbarn $q_1, q_2, ..., q_k$ im Radius $r$ (typisch: 20-50 Nachbarn, nicht nur 3!). Für **jedes Paar** $(p, q_i)$ berechnen wir drei Winkel.

**Warum drei Winkel?** Die relative Lage zweier orientierter Punkte im 3D-Raum hat mehrere Freiheitsgrade:

Ein Winkel allein würde z.B. nicht unterscheiden, ob der Nachbar "links" oder "rechts" von $p$ liegt, oder ob die Normale "nach vorne" oder "zur Seite" zeigt. Die drei Winkel kodieren die vollständige relative Orientierung in einem lokalen Koordinatensystem $(u, v, w)$, das an $p$ verankert ist.

```ascii
Beispiel: Punkt p hat 4 Nachbarn im Radius r

         q₂
          ·
    q₁ ·  p  · q₃        Für jedes Paar (p,q) berechne α, φ, θ:
          ·                 (p,q₁) → α=0.2,  φ=0.8,  θ=1.5
         q₄                 (p,q₂) → α=0.7,  φ=0.3,  θ=0.9
                            (p,q₃) → α=0.3,  φ=0.6,  θ=2.1
                            (p,q₄) → α=0.5,  φ=0.4,  θ=1.2                                                 .
```

Diese Winkelwerte werden nun in **drei separate Histogramme** einsortiert:

```ascii
Histogramm für α:          Histogramm für φ:          Histogramm für θ:
(Wertebereich z.B. -1..1)  (Wertebereich 0..1)        (Wertebereich 0..2π)

Häufigkeit                 Häufigkeit                 Häufigkeit
    │   ▓                      │ ▓                        │     ▓
    │   ▓ ▓                    │ ▓ ▓                      │ ▓   ▓
    │ ▓ ▓ ▓                    │ ▓ ▓ ▓                    │ ▓ ▓ ▓
    └─────────── Bins          └─────────── Bins          └─────────── Bins
     1 2 3 ... 11               1 2 3 ... 11               1 2 3 ... 11                                   .
```

Die drei Histogramme werden konkateniert → **33 Werte** (3 × 11 Bins). Die Bin-Anzahl 11 ist empirisch gewählt (einstellbar in PCL).

**Schritt 3: FPFH (Fast Point Feature Histogram)**

Das "Fast" kommt daher, dass nicht alle Punktpaare betrachtet werden. Stattdessen wird der SPFH von $p$ mit den SPFHs seiner Nachbarn gewichtet kombiniert:

$$FPFH(p) = SPFH(p) + \frac{1}{k} \sum_{i=1}^{k} \frac{1}{d_i} \cdot SPFH(q_i)$$

Nähere Nachbarn ($d_i$ klein) haben mehr Einfluss. So entsteht ein 33-dimensionaler Deskriptor, der die lokale 3D-Geometrie charakterisiert.

**Features:**

+ 33-dimensionaler Histogram-Descriptor
+ Rotation-invariant
+ Robust gegen Rauschen
+ Skaliert gut (schneller als PFH)

**Anwendung: Object Recognition**

```cpp
// Template Matching mit FPFH
// 1. Berechne FPFH für Template
// 2. Berechne FPFH für Szene
// 3. Finde korrespondierende Features (KNN)
// 4. RANSAC für robuste Transformation
// 5. ICP für Fine-Alignment
```

### ICP: Iterative Closest Point

    --{{0}}--
ICP ist der Standardalgorithmus für Point Cloud Registration - das Ausrichten zweier Punktwolken.

**Problem**: Gegeben zwei Punktwolken $P$ und $Q$, finde Transformation $T$, sodass $T(P) \approx Q$

**ICP-Algorithmus:**

```
Initialisierung: T = I (Identität)

Wiederhole bis Konvergenz:
  1. Für jeden Punkt p in P:
     - Finde nächsten Punkt q in Q

  2. Berechne optimale Transformation T':
     - Minimiere Sum of Squared Distances

  3. Update: T = T' · T

  4. Wenn Änderung < threshold: Stop
```

**Mathematik:**

Minimiere:
$$
E(R, t) = \sum_{i=1}^{N} || R p_i + t - q_i ||^2
$$

Wobei:
+ $R$ = Rotationsmatrix (3×3)
+ $t$ = Translationsvektor (3×1)

Lösung: **SVD (Singular Value Decomposition)**

**Anwendungen:**

+ Template Matching (Objekterkennung)
+ SLAM (Scan Matching)
+ 3D-Rekonstruktion
+ Pose Estimation

**Limitierungen:**

+ Benötigt gute Initialisierung
+ Konvergiert zu lokalem Minimum
+ Langsam bei großen Punktwolken

**Verbesserungen:**

+ **Generalized ICP**: Verwendet Plane-to-Plane statt Point-to-Point
+ **Point-to-Plane ICP**: Verwendet Surface Normals
+ **Feature-based Registration**: Erst grobe Ausrichtung mit Features (FPFH), dann ICP

### Deep Learning für 3D: PointNet++

    --{{0}}--
Moderne Ansätze verwenden neuronale Netze direkt auf Punktwolken - ohne Voxelisierung!

**PointNet (2017):**

Erste End-to-End Deep Learning auf Punktwolken

+ Input: $N \times 3$ Matrix (N Punkte mit x, y, z)
+ Permutation-invariant durch Max-Pooling
+ Klassifikation und Segmentation

**PointNet++ (2017):**

Hierarchische Architektur mit lokalem Kontext

```ascii
Input Points         Set Abstraction      Classification
  N × 3               Layers              Output
┌──────┐            ┌──────┐            ┌──────┐
│ · · ·│  ───────>  │ ▓▓▓  │  ───────>  │ Car  │
│· · · │  Group &   │ ▓▓   │  MLP +     │ Tree │
│ · · ·│  Sample    │ ▓    │  Pooling   │ ...  │
└──────┘            └──────┘            └──────┘                                                          .
```

**Vorteile:**

+ Direkte Verarbeitung von Punktwolken
+ Permutation-invariant
+ Robust gegen unterschiedliche Dichten

**Anwendungen:**

+ 3D Object Classification
+ 3D Object Detection (für autonome Fahrzeuge)
+ 3D Semantic Segmentation

**Aktuelle Forschung (2024):**

+ **Point Transformer**: Self-Attention für Punktwolken
+ **VoxelNet**: Hybrid aus Voxeln und PointNet
+ **BEVFusion**: Multi-Modal (Camera + Lidar) für autonome Fahrzeuge

> Deep Learning für 3D ist ein aktives Forschungsgebiet - die Methoden entwickeln sich rasant!

## Zusammenfassung & Ausblick

    --{{0}}--
Wir haben heute eine umfassende Einführung in Objekterkennung und Tracking erhalten - von klassischen Feature-basierten Methoden bis zu modernem Deep Learning, von 2D-Bildern bis zu 3D-Punktwolken.

### Was haben wir gelernt?

Die Vorlesung spannte einen Bogen von klassischen bis hin zu modernen Verfahren der Objekterkennung:

**Klassische Feature-Methoden** bilden die Grundlage für geometrische Anwendungen wie SLAM und visuelle Odometrie. Mit Harris und Shi-Tomasi haben wir Corner-Detektoren kennengelernt, die mathematisch fundiert arbeiten und ohne Training auskommen. ORB kombiniert schnelle Detektion (FAST) mit kompakten binären Deskriptoren (rBRIEF).

**Deep Learning** ermöglicht semantisches Verständnis von Szenen. YOLOv8 erkennt Objekte in Echtzeit und liefert die Bounding Boxes, die wir in Übung 2 mit Stereo-Tiefendaten kombinieren werden. Die Vortrainierung auf COCO macht den Einstieg einfach - gleichzeitig sollte man die Grenzen dieser "Black Box"-Modelle im Blick behalten.

**Tracking** erweitert die Einzelbild-Detektion um zeitliche Konsistenz. Der Lucas-Kanade Algorithmus verfolgt Features zwischen Frames durch Gradientenanalyse, während DeepSORT komplexe Szenen mit mehreren Objekten handhabt und dabei IDs über Verdeckungen hinweg erhält.

**3D-Verfahren** nutzen die Punktwolken aus Lidar oder Stereo-Kameras. RANSAC segmentiert robuste Ebenen (etwa den Boden), Euclidean Clustering separiert Objekte, und ICP aligniert Punktwolken - wichtig für Scan-Matching in SLAM. Mit PointNet++ haben wir einen Ausblick auf Deep Learning direkt auf 3D-Daten gegeben.


### Ausblick: Nächste Vorlesung

**L09: Sensordatenfusion I - Grundlagen**

+ Warum mehrere Sensoren kombinieren?
+ Diskrete Bayes-Filter
+ Komplementärfilter (IMU-Fusion)
+ Fehlerfortpflanzung und Unsicherheiten
+ Vorbereitung auf Kalman-Filter