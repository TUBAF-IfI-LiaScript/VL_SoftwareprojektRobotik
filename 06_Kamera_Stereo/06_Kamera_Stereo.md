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

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/06_Kamera_Stereo/06_Kamera_Stereo.md)

# Kameramodelle und Stereo-Vision

| Parameter            | Kursinformationen                                                                                                     |
| -------------------- | --------------------------------------------------------------------------------------------------------------------- |
| **Veranstaltung:**   | @config.lecture                                                                                                       |
| **Semester**         | @config.semester                                                                                                      |
| **Hochschule:**      | `Technische Universität Freiberg`                                                                                     |
| **Inhalte:**         | `Kameramodelle, Kalibrierung und Stereo-Vision`                                                                       |
| **Link auf GitHub:** | https://github.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/blob/master/06_Kamera_Stereo/06_Kamera_Stereo.md |
| **Autoren**          | @author                                                                                                               |

![](https://media4.giphy.com/media/v1.Y2lkPTc5MGI3NjExN2NrbThydjk3bmRjb3ZpbzZ4NTY3d2ZpNTUwMzU1M2NmMmE3aGswdSZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/2RZCJlQmolMru/giphy.gif)

--------------------------------------------------------------------------------

**Zielstellung der heutigen Veranstaltung**

+ Verständnis des Pinhole-Kameramodells
+ Intrinsische und extrinsische Kameraparameter
+ Kamerakalibrierung mit OpenCV
+ Verzerrungskorrektur
+ Prinzipien der Stereo-Vision
+ Disparitätsberechnung und Tiefenschätzung
+ PointCloud2-Messages in ROS 2
+ Praktische Anwendungen in der mobilen Robotik

--------------------------------------------------------------------------------

## Motivation: Von 2D zu 3D

    --{{0}}--
Willkommen zur siebten Vorlesung! In der letzten Veranstaltung haben wir grundlegende Bildverarbeitungstechniken kennengelernt. Heute machen wir den nächsten Schritt: Wir lernen, wie Roboter mit Kameras die dreidimensionale Struktur ihrer Umgebung erfassen können.

Eine Kamera bildet die dreidimensionale Welt auf eine zweidimensionale Bildebene ab. Dabei gehen Informationen verloren!

```ascii
3D-Welt                    Kamera                  2D-Bild
                          ┌─────┐
   •  A (groß, nah)       │     │                    • a
                          │  ●──┼──────────────────► (klein)
   •  B (klein, fern)     │     │
                          └─────┘

Problem: A und B können auf denselben Bildpunkt abgebildet werden!
         → Tiefeninformation geht verloren                                                                      .
```

**Herausforderungen:**

+ **Größen-Distanz-Ambiguität**: Ein großes, entferntes Objekt kann genauso erscheinen wie ein kleines, nahes Objekt
+ **Verlust der Tiefe**: Aus einem einzelnen Bild können wir nicht direkt auf Entfernungen schließen
+ **Perspektivische Verzerrung**: Parallele Linien scheinen sich in der Ferne zu treffen


| Anwendung                     | Technologie              | Beispiel                                   |
| ----------------------------- | ------------------------ | ------------------------------------------ |
| **Hinderniserkennung**        | Stereo-Vision            | Autonome Fahrzeuge erkennen Fußgänger      |
| **Greifplanung**              | 3D-Rekonstruktion        | Roboterarm greift Objekte                  |
| **Navigation**                | Visual Odometry          | Drohnen navigieren in GPS-freien Umgebungen|
| **3D-Kartierung**             | SLAM mit Tiefenkameras   | Mobile Roboter erstellen 3D-Karten         |
| **Vermessung**                | Photogrammetrie          | Inspektion von Bauwerken                   |

> **Beispiel**: Autonome Fahrzeuge müssen präzise Entfernungen zu Objekten messen, um sicher zu bremsen!

## Das 2D Pinhole-Kameramodell

    --{{0}}--
Das Pinhole- oder Lochkameramodell ist das grundlegende mathematische Modell, das beschreibt, wie eine Kamera 3D-Punkte auf die 2D-Bildebene projiziert.

**Konzept**: Licht fällt durch eine kleine Öffnung (Pinhole) und erzeugt ein umgekehrtes Bild auf der Bildebene

![Pinhole-Kamera Prinzip](https://upload.wikimedia.org/wikipedia/commons/thumb/3/3b/Pinhole-camera.svg/800px-Pinhole-camera.svg.png "Prinzip einer Lochkamera - Von Pbroks13 - Eigenes Werk, CC BY 3.0, https://commons.wikimedia.org/w/index.php?curid=3217146")<!-- style="width: 40%;" -->


**Koordinatensystem:**

+ Kamerazentrum (optisches Zentrum) = Ursprung
+ Z-Achse = optische Achse (in Blickrichtung)
+ Bildebene im Abstand f (Brennweite) vor dem Zentrum
+ X-Achse zeigt nach rechts, Y-Achse nach unten (Bildkoordinaten-Konvention)

> Achtung: In der realen Kamera befindet sich die Bildebene hinter dem Pinhole, was zu einem umgekehrten Bild führt. Für die mathematische Modellierung platzieren wir die Bildebene jedoch vor dem Pinhole, um die Gleichungen zu vereinfachen.

> Achtung: Das Pinhole-Modell ist eine Idealisierung. Reale Kameras haben Linsen, die Verzerrungen verursachen können! Siehe den Abschnitt zur Kalibrierung ...

### Abbildung eines 3D-Punkts auf die Bildebene

![Pinhole-Kamera-Modell](https://upload.wikimedia.org/wikipedia/commons/thumb/7/7c/Pinhole.svg/2560px-Pinhole.svg.png "Pinhole-Kamera-Modell mit virtueller Bildebene - en:User:KYNen:User:N3bulous - en:File:Pinhole.svg, Public Domain, https://en.wikipedia.org/wiki/Pinhole_camera_model#/media/File:Pinhole.svg")<!-- style="width: 50%;" -->

**Mathematische Herleitung anhand der Abbildung:**

Wie in der Abbildung zu sehen, verwenden wir eine **virtuelle Bildebene** vor dem Pinhole (nicht dahinter). Dies vermeidet das umgekehrte Bild und vereinfacht die Mathematik erheblich.

**Koordinatensystem und Bezeichnungen (gemäß Abbildung):**

+ **Pinhole** (optisches Zentrum) = Koordinatenursprung C
+ **Objektebene**: Punkte werden mit $X_1, X_2, X_3$ bezeichnet (3D-Koordinaten)
+ **Bildebene**: Punkte werden mit $Y_1, Y_2$ bezeichnet (2D-Koordinaten)
+ **Brennweite f**: Abstand zwischen Pinhole und virtueller Bildebene

**Von 3D (Objektebene) zu 2D (Bildebene)**

Gegeben: 3D-Punkt auf der Objektebene mit Koordinaten $(X_1, X_2, X_3)$

Gesucht: 2D-Bildpunkt auf der Bildebene mit Koordinaten $(Y_1, Y_2)$

**Aus ähnlichen Dreiecken (siehe Abbildung):**

Die gestrichelten Linien in der Abbildung zeigen die Projektionsstrahlen vom 3D-Punkt durch das Pinhole zur Bildebene. Durch ähnliche Dreiecke ergibt sich:

$$
\frac{Y_1}{f} = \frac{X_1}{X_3} \quad \text{und} \quad \frac{Y_2}{f} = \frac{X_2}{X_3}
$$

**Projektionsgleichung:**

$$
Y_1 = f \cdot \frac{X_1}{X_3}, \quad Y_2 = f \cdot \frac{X_2}{X_3}
$$

> **Wichtig**: Je größer $X_3$ (Tiefe), desto kleiner erscheint der Punkt im Bild!

### Homogene Koordinaten und Matrixform

**Problem**: Die Projektionsgleichungen sind nicht-linear (Division durch $X_3$) → schlecht für Matrixoperationen und Koordinatentransformationen

**Lösung**: Homogene Koordinaten ermöglichen eine **einheitliche Matrixdarstellung**

In homogenen Koordinaten wird ein 2D-Punkt $(Y_1, Y_2)$ als 3D-Vektor $[Y_1, Y_2, 1]^T$ dargestellt.

**Projektionsmatrix (vereinfacht):**

$$
\begin{bmatrix} Y_1 \\ Y_2 \\ 1 \end{bmatrix}
\sim
\begin{bmatrix}
f & 0 & 0 & 0 \\
0 & f & 0 & 0 \\
0 & 0 & 1 & 0
\end{bmatrix}
\begin{bmatrix} X_1 \\ X_2 \\ X_3 \\ 1 \end{bmatrix}
=
\begin{bmatrix} f X_1 \\ f X_2 \\ X_3 \end{bmatrix}
$$

Nach **Normalisierung** (Division durch $X_3$):

$$
\begin{bmatrix} Y_1 \\ Y_2 \\ 1 \end{bmatrix}
=
\begin{bmatrix} f X_1/X_3 \\ f X_2/X_3 \\ 1 \end{bmatrix}
$$

> **Vorteil**: Diese Matrixform ist die Grundlage für Kamera-Matrix K und Koordinatentransformationen (siehe folgende Abschnitte)

## Abweichungen realer Kameras vom Pinhole-Modell

![](https://upload.wikimedia.org/wikipedia/commons/thumb/b/b3/Optische_Abbildungsgeometrie_einer_Kamera.svg/1024px-Optische_Abbildungsgeometrie_einer_Kamera.svg.png "Optische Abbildungsgeometrie einer Kamera - Von Olaf Peters (OlafTheScientist) - Eigenes Werk, CC BY-SA 4.0, https://commons.wikimedia.org/w/index.php?curid=91718699")<!-- style="width: 50%;" -->

| **Aspekt** | **Pinhole-Modell (Ideal)** | **Reale Kamera** | **Auswirkung** |
|------------|----------------------------|-------------------|----------------|
| **Öffnung** | Unendlich kleine Punktöffnung | Blende mit endlicher Größe | Tiefenschärfe-Effekte |
| **Optik** | Keine Linsen | Komplexes Linsensystem | Verzerrungen, Aberrationen |
| **Projektionszentrum** | Ein einziges Zentrum $O$ | Objektseitiges $(O)$ und bildseitiges $(O')$ Projektionszentrum | Verschiedene Brennweiten |
| **Hauptpunkt** | Bildmitte (ideal) | Hauptpunkt $H$ (senkrechte Projektion von $O'$) | Verschiebung des optischen Zentrums |
| **Optische Achse** | Exakt senkrecht zur Bildebene | Oft nicht senkrecht | Symmetriepunkt der Verzeichnung $S \neq H$ |
| **Winkelbeziehung** | $\tau = \tau'$ (winkeltreu) | $\tau \neq \tau'$ | Verzeichnung (barrel/pincushion) |
| **Strahlenverlauf** | Ein Strahl pro Objektpunkt | Strahlenkegel durch Blende | Unschärfe, Beugungseffekte |
| **Kamerakonstante** | Brennweite $f$ | Kamerakonstante $c$ (Abstand $O'$ zur Bildebene) | Unterschiedliche Kalibrierwerte |

**Wichtige Parameter realer Kameras:**

+ **Eintrittspupille (EP)**: Objektseitiges Bild der Blende
+ **Austrittspupille (AP)**: Bildseitiges Bild der Blende  
+ **Hauptstrahl**: Repräsentativer Strahl durch Projektionszentrum $O$
+ **Strahlenkegel**: Alle Strahlen eines Objektpunkts durch die Blende

> Diese Abweichungen führen zu den **Verzerrungsparametern**, die bei der Kamerakalibrierung bestimmt werden müssen.

| Komponente                     | Zweck                    | Parameter                 | Modelliert                |
| ------------------------------ | ------------------------ | ------------------------- | ------------------------- |
| **Kamera-Matrix K**            | Lineare Projektion       | $f_x, f_y, c_x, c_y, s$   | Ideale Pinhole-Projektion |
| **Verzerrungskoeffizienten D** | Nichtlineare Korrekturen | $k_1, k_2, k_3, p_1, p_2$ | Linsenverzerrungen        |

### Die Kamera-Matrix K

**Die K-Matrix** modelliert nur die **ideale geometrische Projektion**:

$$
K = \begin{bmatrix}
f_x & s & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
$$

**Parameter:**

<!-- data-type="none" -->
| Parameter  | Bedeutung                          | Typischer Wert | Einheit |
| ---------- | ---------------------------------- | -------------- | ------- |
| $f_x, f_y$ | Brennweite in Pixel (x/y-Richtung) | 500-1500       | Pixel   |
| $c_x, c_y$ | Hauptpunkt (Principal Point)       | Bildmitte      | Pixel   |
| $s$        | Skew (Scherung), meist 0           | 0              | Pixel   |

**Erklärung:**

+ **Brennweite in Pixel**: $f_x = f \cdot m_x$, wobei $f$ die physische Brennweite (mm) und $m_x$ die Pixeldichte (Pixel/mm) ist
+ **Hauptpunkt**: Schnittpunkt der optischen Achse mit der Bildebene (idealerweise Bildmitte)
+ **Skew**: Beschreibt nicht-rechtwinklige Pixel (bei modernen Kameras meist 0)

**Vollständige Projektion:**

$$
K \begin{bmatrix} X \\ Y \\ Z \end{bmatrix}
=
\begin{bmatrix}
f_x & s & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix} X \\ Y \\ Z \end{bmatrix}
$$

wobei $(u, v)$ die **Pixelkoordinaten** im Bild sind.

**Die Kamera-Matrix K erfasst nur einen Teil der realen Kamera-Abweichungen:**

| **Parameter aus Tabelle** | **In K enthalten?** | **K-Parameter** | **Erklärung** |
|---------------------------|---------------------|-----------------|---------------|
| **Hauptpunkt H** | ✅ **Ja** | $(c_x, c_y)$ | Verschiebung des optischen Zentrums |
| **Kamerakonstante c** | ✅ **Ja** | $(f_x, f_y)$ | Brennweite in Pixelkoordinaten |
| **Skew/Scherung** | ✅ **Ja** | $s$ | Nicht-rechtwinklige Pixel |
| **Verschiedene Brennweiten** | ✅ **Ja** | $f_x \neq f_y$ | Unterschiedliche Skalierung in x/y |
| **Projektionszentren O, O'** | ❌ **Nein** | - | Vereinfacht zu einem Zentrum |
| **Verzeichnung ($\tau \neq \tau'$)** | ❌ **Nein** | - | **Separate Verzerrungsparameter nötig** |
| **Symmetriepunkt S ≠ H** | ❌ **Nein** | - | **Separate Verzerrungsparameter nötig** |
| **Strahlenkegel-Effekte** | ❌ **Nein** | - | Tiefenschärfe, optische Effekte |

> **Wichtig**: Die Kamera-Matrix K modelliert die **lineare Projektion**, aber **nicht** die **Verzerrungen**!

**Deshalb brauchen wir zusätzlich:**

+ **Verzerrungskoeffizienten** $(k_1, k_2, k_3, p_1, p_2)$ für radiale und tangentiale Verzeichnung
+ **Separate Kalibrierung** zur Bestimmung von K und Verzerrungsparametern

### Verzerrungsparameter

**Die Verzerrungskoeffizienten** sind der **zweite Teil** der intrinsischen Parameter:

Reale Linsen verursachen **geometrische Verzerrungen**:

**1. Radiale Verzerrung** – abhängig vom Abstand zum Bildmittelpunkt

![Linsenverzerrung](https://upload.wikimedia.org/wikipedia/commons/thumb/3/31/Lens_distorsion.svg/1280px-Lens_distorsion.svg.png "Stellung der Blende verursacht kissenförmige Verzeichnung (oben), tonnenförmige Verzeichnung (Mitte), keine Verzeichnung (unten) - Von Nicoguaro - Eigenes Werk, CC BY-SA 3.0, https://commons.wikimedia.org/w/index.php?curid=17027248")<!-- style="width: 50%;" -->

+ **Tonnenverzerrung** (barrel distortion): $k_1 < 0$ – häufig bei Weitwinkelobjektiven
+ **Kissenverzerzung** (pincushion distortion): $k_1 > 0$ – häufig bei Teleobjektiven

**Modell:**

$$
\begin{align}
x_{\text{verzerrt}} &= x (1 + k_1 r^2 + k_2 r^4 + k_3 r^6) \\
y_{\text{verzerrt}} &= y (1 + k_1 r^2 + k_2 r^4 + k_3 r^6)
\end{align}
$$

wobei $r^2 = x^2 + y^2$ (Abstand vom Hauptpunkt, normalisiert)

**2. Tangentiale Verzerrung** – durch nicht-parallele Linsenebenen

$$
\begin{align}
x_{\text{verzerrt}} &= x + [2p_1 xy + p_2(r^2 + 2x^2)] \\
y_{\text{verzerrt}} &= y + [p_1(r^2 + 2y^2) + 2p_2 xy]
\end{align}
$$

**Verzerrungskoeffizienten:**

$$
D = (k_1, k_2, p_1, p_2, k_3)
$$

> Bei Weitwinkel- und Fischaugenobjektiven ist die Verzerrung besonders stark!

### Anwendung der intrinsischen Parameter

> Wie werden K-Matrix und Verzerrungskoeffizienten verwendet, um ein verzerrtes Kamerabild zu entzerren?**

**Schritt 1: Vom verzerrten Bild zu normalisierten Koordinaten**

```ascii
Verzerrtes Kamerabild        K⁻¹ (inverse K-Matrix)      Normalisierte Koordinaten
     [u_verzerrt]                                              [x_verzerrt]
     [v_verzerrt]        →        K⁻¹ × [u v 1]ᵀ        =      [y_verzerrt]
     [    1     ]                                              [    1     ]
```

**Mathematisch:**

$$
\begin{bmatrix} x_{\text{verzerrt}} \\ y_{\text{verzerrt}} \\ 1 \end{bmatrix} = K^{-1} \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
$$

**Schritt 2: Verzerrung rückgängig machen**

Die **verzerrten Koordinaten** $(x_{\text{verzerrt}}, y_{\text{verzerrt}})$ werden zu **idealisierten Koordinaten** $(x_{\text{ideal}}, y_{\text{ideal}})$ korrigiert:

```ascii
Verzerrte Koordinaten    Umkehr-Verzerrungsmodell    Idealisierte Koordinaten
   (x_verzerrt, y_verzerrt)                              (x_ideal, y_ideal)
           |                                                     ^
           |        Iterative Lösung von:                        |
           |   x_verzerrt = x_ideal(1 + k₁r² + k₂r⁴)             |
           |             + [2p₁x_ideal·y_ideal + p₂(r² + 2x²)]   |
           └─────────────────────────────────────────────────────┘
```

**Achtung**: Diese Umkehrung ist **nicht analytisch lösbar** → iterative Methoden oder Lookup-Tabellen!

**Schritt 3: Zurück zu entzerrten Pixelkoordinaten**

Die idealisierten Koordinaten werden in **entzerrte Pixelkoordinaten** umgewandelt:

$$
\begin{align}
u_{\text{entzerrt}} &= f_x \cdot x_{\text{ideal}} + c_x \\
v_{\text{entzerrt}} &= f_y \cdot y_{\text{ideal}} + c_y
\end{align}
$$


```python
import cv2
import numpy as np

def undistort_image(distorted_img, K, dist_coeffs):

    
    return undistorted_img

# Beispiel-Verwendung
K = np.array([[533.3, 0, 320.5],
              [0, 533.3, 240.5],  
              [0, 0, 1]], dtype=np.float32)

dist_coeffs = np.array([-0.28, 0.11, 0.001, 0.0005, -0.04], dtype=np.float32)

# Lade verzerrtes Bild
distorted = cv2.imread('raw_camera_image.jpg')

# Wende Entzerrung an - Entzerrt ein Kamerabild mit intrinsischen Parametern
# 
# Args:
#     distorted_img: Verzerrtes Eingangsbild
#     K: 3x3 Kamera-Matrix
#     dist_coeffs: [k1, k2, p1, p2, k3] Verzerrungskoeffizienten
# 
# Returns:
#     undistorted_img: Entzerrtes Ausgangsbild
undistorted_img = cv2.undistort(distorted_img, K, dist_coeffs)

# Speichere Ergebnis
cv2.imwrite('corrected_camera_image.jpg', undistorted)
```

## Kamerakalibrierung

Kalibrierung ist der Prozess, bei dem wir 

+ die intrinsischen Parameter K und die Verzerrungskoeffizienten D einer Kamera bestimmen und / oder
+ ggf. die extrinsischen Parameter (Rotation R und Translation t) relativ zu einer Weltkoordinate schätzen.

**Ohne Kalibrierung:**

+ ❌ Verzerrte Bilder
+ ❌ Falsche Entfernungsmessungen
+ ❌ Ungenaue 3D-Rekonstruktion

### Intrinsische Kalibrierung mit Schachbrettmuster

**Idee**: Fotografiere ein bekanntes Muster aus verschiedenen Perspektiven

![](https://upload.wikimedia.org/wikipedia/commons/thumb/0/05/Multiple_chessboard_views.png/960px-Multiple_chessboard_views.png "Verschiedene Ansichten eines Schachbretts zur Kamerakalibrierung - Von Brian moore81 - Eigenes Werk, CC BY-SA 4.0, https://commons.wikimedia.org/w/index.php?curid=37007184")

**Ablauf:**

1. Vorbereitung eines verwindungssteifen Schachbrettmuster bekannter Größe 
2. Nimm 10-20 Bilder aus verschiedenen Winkeln und Entfernungen auf
3. Erkenne automatisch die Schachbrett-Ecken in jedem Bild
4. Berechne K und D durch Optimierung

**OpenCV-Implementierung:**

```python
import cv2
import numpy as np
import glob

# 1. Schachbrett-Spezifikation
CHECKERBOARD = (6, 9)  # Innere Ecken (Höhe, Breite)
square_size = 0.025    # 25mm in Metern

# 3D-Punkte des Schachbretts (in Weltkoordinaten)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

# Listen für 3D-Punkte und 2D-Bildpunkte
objpoints = []  # 3D-Punkte in der realen Welt
imgpoints = []  # 2D-Punkte im Bild

# 2. Lade Kalibrierbilder
images = glob.glob('calibration_images/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 3. Finde Schachbrett-Ecken
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)

        # Verfeinere Ecken-Positionen (sub-pixel)
        corners_refined = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        imgpoints.append(corners_refined)

        # Zeichne Ecken (zur Kontrolle)
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners_refined, ret)
        cv2.imshow('Corners', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# 4. Kalibrierung durchführen
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("Kamera-Matrix K:")
print(K)
print("\nVerzerrungskoeffizienten:")
print(dist)

# Speichern
np.savez('camera_calibration.npz', K=K, dist=dist)
```

**Ausgabe-Beispiel:**

```
Kamera-Matrix K:
[[532.8  0.0  342.5]
 [  0.0 532.8 233.1]
 [  0.0   0.0   1.0]]

Verzerrungskoeffizienten:
[[-0.28  0.11  0.001  0.0005  -0.04]]
 k1     k2     p1      p2       k3
```

### Extrinsische Parameter

Während intrinsische Parameter die Kamera selbst beschreiben, beschreiben extrinsische Parameter die Position und Orientierung der Kamera in der Welt.

**Extrinsische Parameter:**

+ **Rotation R** (3×3-Matrix): Orientierung der Kamera
+ **Translation t** (3×1-Vektor): Position der Kamera

> Auch extrinsische Parameter werden bei der Kalibrierung bestimmt! Dazu kann wiederum unser Schachbrettmuster verwendet werden.

## Stereo-Vision

    --{{0}}--
Mit einer einzelnen Kamera können wir keine Tiefe messen. Die Lösung: Zwei Kameras, die wie unsere Augen funktionieren!

### Prinzip der Stereo-Vision

**Konzept**: Zwei Kameras mit bekanntem Abstand (Baseline) betrachten dieselbe Szene

![](https://upload.wikimedia.org/wikipedia/commons/b/b9/Stereoscopic_images%2C_depth_to_displacement_relationship_assuming_flat_co-planar_images..png "Beziehung zwischen Tiefe und Disparität in der Stereo-Vision - By Thepigdog - Own work, CC BY-SA 3.0, https://commons.wikimedia.org/w/index.php?curid=32757926")<!-- style="width: 30%;" -->

> Betrachten Sie die zwei ähnlichen Dreiecke in der Abbildung oben. Durch die bekannte Baseline und die gemessene Disparität können wir die Tiefe berechnen.

$$
\begin{align}
\frac{BC + CD}{AC} &= \frac{EF}{BF} + \frac{GH}{DG} \\
\frac{BC + CD}{AC} &= \frac{EF}{BF} + \frac{GH}{BF} \\
\frac{BC + CD}{AC} &= \frac{1}{BF}(EF + GH) \\
AC &= \frac{BF (BC + CD)}{EF + GH} \\
z &= \frac{f \cdot b}{d} \\
\end{align}
$$

wobei:
- $BC + CD = BD = b$ (Baseline)
- $EF + GH = d$ (Disparität) 
- $BF = f$ (Brennweite)
- $AC = z$ (gesuchte Distanz)

**Menschliche Analogie**: Unsere Augen haben einen Abstand von ~6-7 cm

> Welche Auswirkungen hat die Baseline auf die Tiefenmessung?

+ **Große Disparität** ($d \uparrow$) → **kleine Distanz** ($z \downarrow$) → **nahes Objekt**
+ **Kleine Disparität** ($d \downarrow$) → **große Distanz** ($z \uparrow$) → **fernes Objekt**  
+ **Größere Baseline** ($b \uparrow$) → **bessere Tiefenauflösung**
+ **Größere Brennweite** ($f \uparrow$) → **bessere Tiefengenauigkeit**

### Vorverarbeitung

Stereo-Rektifizierung
========================

**Problem**: Kameras sind nie perfekt ausgerichtet

**Lösung**: **Rektifizierung** – virtuelle Rotation der Bilder, sodass:

1. Bildzeilen parallel sind
2. Korrespondierende Punkte auf derselben Zeile liegen

```ascii
Vor Rektifizierung:             Nach Rektifizierung:

Linkes Bild   Rechtes Bild      Linkes Bild   Rechtes Bild
  ●  .                             ──●────────────●──  ← Epipolarlinie
     ●.                              ─────●────●─────
       .●
                                   Punkte liegen auf
Punkt liegt auf                    derselben Zeile!
schräger Epipolarlinie                                                                                      .
```

**Vorteile der Rektifizierung:**

+ ✅ Vereinfachte Suche nach Korrespondenzen (nur entlang einer Zeile)
+ ✅ Schnellere Stereo-Matching-Algorithmen
+ ✅ Geringerer Rechenaufwand


Stereo-Matching
========================

**Ziel**: Für jeden Pixel im linken Bild, finde den korrespondierenden Pixel im rechten Bild

![](https://docs.opencv.org/4.x/matcher_result1.jpg "" "Beispiel für Stereo-Matching - Von OpenCV Documentation - https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html")<!-- style="width: 50%;" -->

Als Beispielhafte Umsetzung sei hier der ORB (Oriented FAST and Rotated BRIEF) Ansatz erwähnt, er verwendet den FAST-Algorithmus (Features from Accelerated Segment Test), um interessante Punkte im Bild zu identifizieren. FAST sucht nach Pixeln, die sich deutlich von ihren umliegenden Pixeln unterscheiden (z.B. Ecken).

| Problem | Ursache | Lösung |
|---------|---------|--------|
| **Texturlose Bereiche** | Keine eindeutigen Features | Multi-View, ML-Methoden |
| **Okklusionen** | Objekt nur in einem Bild sichtbar | Left-Right Consistency Check |
| **Reflexionen** | Spiegelnde Oberflächen | Strukturiertes Licht |
| **Rechenaufwand** | Viele Pixel zu vergleichen | GPU-Beschleunigung, ROI |


## ROS 2 Integration: PointCloud2


Nachdem wir nun verstehen, wie Stereo-Vision funktioniert, schauen wir uns an, wie die resultierenden 3D-Daten in ROS 2 repräsentiert werden.

### Das PointCloud2-Message-Format

**PointCloud2** ist das Standard-ROS-2-Format für 3D-Punktwolken

**Message-Definition:**

```bash
# sensor_msgs/msg/PointCloud2

std_msgs/Header header
  uint32 height      # Höhe der Punktwolke (bei organisierten Wolken)
  uint32 width       # Breite der Punktwolke

PointField[] fields  # Beschreibt die Datenfelder (x, y, z, rgb, etc.)

bool    is_bigendian # Byte-Reihenfolge
uint32  point_step   # Länge eines Punkts in Bytes
uint32  row_step     # Länge einer Zeile in Bytes
uint8[] data         # Rohdaten (alle Punkte serialisiert)
bool    is_dense     # true = keine ungültigen Punkte (inf/NaN)
```

**PointField-Struktur:**

```bash
# sensor_msgs/msg/PointField

string name      # Feldname (z.B. "x", "y", "z", "rgb")
uint32 offset    # Byte-Offset vom Start des Punkts
uint8  datatype  # Datentyp (INT8=1, UINT8=2, FLOAT32=7, etc.)
uint32 count     # Array-Größe (meist 1)
```

**Das Problem**: Das `data[]`-Array ist nur ein **flacher Byte-Strom** ohne Struktur!

`data[] = [0x42, 0x48, 0x00, 0x00, 0x43, 0x96, 0x00, 0x00, ...]`
         
**PointField als "Decoder-Anleitung":**

```python
# Beispiel: XYZ + RGB PointCloud
fields = [
    PointField(name="x",   offset=0,  datatype=7, count=1),  # FLOAT32 
    PointField(name="y",   offset=4,  datatype=7, count=1),  # FLOAT32
    PointField(name="z",   offset=8,  datatype=7, count=1),  # FLOAT32  
    PointField(name="rgb", offset=12, datatype=6, count=1)   # UINT32
]

# point_step = 16 (4+4+4+4 Bytes pro Punkt)
```

**Interpretation des data[]-Arrays:**

```ascii
Punkt 0 (Bytes 0-15):           Punkt 1 (Bytes 16-31):
┌─────┬─────┬─────┬─────┐      ┌─────┬─────┬─────┬─────┐
│ x   │ y   │ z   │ rgb │      │ x   │ y   │ z   │ rgb │
│ 0-3 │ 4-7 │8-11 │12-15│      │16-19│20-23│24-27│28-31│
│float│float│float│uint │      │float│float│float│uint │
└─────┴─────┴─────┴─────┘      └─────┴─────┴─────┴─────┘                                                     .
```

**Praktisches Beispiel: Daten extrahieren**

```python
import struct
from sensor_msgs.msg import PointCloud2, PointField

def extract_point(data, point_index, fields, point_step):
    """
    Extrahiere einen Punkt aus dem data-Array
    """
    # Startposition für diesen Punkt
    start_byte = point_index * point_step
    point_data = {}
    
    for field in fields:
        # Byte-Position für dieses Feld
        field_start = start_byte + field.offset
        field_end = field_start + 4  # Annahme: 4 Bytes pro Feld
        
        # Bytes extrahieren
        field_bytes = data[field_start:field_end]
        
        # Je nach Datentyp interpretieren
        if field.datatype == 7:  # FLOAT32
            value = struct.unpack('f', field_bytes)[0]
        elif field.datatype == 6:  # UINT32  
            value = struct.unpack('I', field_bytes)[0]
        # ... weitere Datentypen
        
        point_data[field.name] = value
    
    return point_data

# Verwendung:
point_0 = extract_point(msg.data, 0, msg.fields, msg.point_step)
print(f"Punkt 0: x={point_0['x']}, y={point_0['y']}, z={point_0['z']}")
```

**Warum ist PointField so wichtig?**

1. **Flexibilität**: Verschiedene Punktformate (XYZ, XYZRGB, XYZI, etc.)
2. **Interoperabilität**: Verschiedene Software kann denselben Datenstream verstehen  
3. **Effizienz**: Kompakte Speicherung ohne Overhead
4. **Erweiterbarkeit**: Neue Felder können einfach hinzugefügt werden

**Praktische RGB-Kodierung:**

```python
# RGB-Wert erstellen
def pack_rgb(r, g, b, a=255):
    """
    Packt RGB(A)-Werte in uint32
    
    Args:
        r, g, b: Rot, Grün, Blau (0-255)
        a: Alpha/Transparenz (0-255, Standard: 255=undurchsichtig)
    
    Returns:
        uint32: Gepackter RGB-Wert
    """
    return (a << 24) | (r << 16) | (g << 8) | b

# RGB-Wert extrahieren  
def unpack_rgb(rgb_uint32):
    """
    Extrahiert RGB(A) aus uint32
    
    Returns:
        tuple: (r, g, b, a)
    """
    a = (rgb_uint32 >> 24) & 0xFF
    r = (rgb_uint32 >> 16) & 0xFF  
    g = (rgb_uint32 >> 8) & 0xFF
    b = rgb_uint32 & 0xFF
    return r, g, b, a

# Beispiele:
rot = pack_rgb(255, 0, 0)      # 0xFF0000FF
gruen = pack_rgb(0, 255, 0)    # 0xFF00FF00  
blau = pack_rgb(0, 0, 255)     # 0xFFFF0000
schwarz = pack_rgb(0, 0, 0)    # 0xFF000000
```

### Kameraparameter in ROS2

**Kameraparameter** werden in ROS 2 typischerweise über das `sensor_msgs/CameraInfo`-Message übertragen.

```bash
# sensor_msgs/msg/CameraInfo
std_msgs/Header header
uint32 height            # Bildhöhe         
uint32 width             # Bildbreite
string distortion_model  # Verzerrungsmodell (z.B. "plumb_bob")
float64[] D              # Verzerrungskoeffizienten
float64[9] K             # Kamera-Matrix
float64[9] R             # Rotationsmatrix      
float64[12] P            # Projektionsmatrix
```
`sensor_msgs/CameraInfo` enthält alle notwendigen intrinsischen Parameter für die Bildentzerrung und 3D-Rekonstruktion.

> Die Parameter einer Kamera können als Konfigurationsdatei (YAML) gespeichert und beim Start des ROS 2 Knotens geladen werden.

## Herausforderungen bei der Arbeit mit Punktwolken

Bei der praktischen Arbeit mit Punktwolken in der Robotik stoßen wir auf verschiedene Herausforderungen. Diese zu kennen hilft uns, robustere Systeme zu entwickeln und Probleme frühzeitig zu erkennen.

### 1. Datenmenge und Performance

**Problem**: Punktwolken können sehr groß werden

```ascii
Full HD Stereo (1920×1080):
    Maximale Punktzahl = 1920 × 1080 = 2.073.600 Punkte
    Pro Punkt: 16 Bytes (x, y, z, rgb als float32/uint32)
    Gesamtgröße ≈ 33 MB pro Frame!

    Bei 30 fps: ~1 GB/s Datenrate                                                                                 .
```

**Auswirkungen:**

+ ⚠️ Hohe CPU/GPU-Last
+ ⚠️ Speicherbedarf
+ ⚠️ Netzwerk-Überlastung (bei ROS 2 über WLAN)
+ ⚠️ Verzögerte Verarbeitung

**Lösungsansätze:**

| Ansatz            | Beschreibung                              | Vor-/Nachteile                          |
| ----------------- | ----------------------------------------- | --------------------------------------- |
| **Downsampling**  | Reduziere Auflösung                       | ✅ Schnell, ❌ Verlust an Details       |
| **Voxel-Filter**  | Rastere in 3D-Gitter                      | ✅ Gleichmäßig, ✅ Weniger Rauschen     |
| **ROI-Filterung** | Nur relevanter Bereich                    | ✅ Sehr effektiv, ❌ Benötigt Vorwissen |
| **Kompression**   | Verlustfreie/verlustbehaftete Kompression | ✅ Für Netzwerk, ❌ CPU-Overhead        |

**Beispiel: Voxel-Downsampling mit Open3D**

```python
import open3d as o3d
import numpy as np
from sensor_msgs_py import point_cloud2

class VoxelDownsampler(Node):
    def __init__(self):
        super().__init__('voxel_downsampler')

        self.declare_parameter('voxel_size', 0.05)  # 5cm Voxel-Größe
        voxel_size = self.get_parameter('voxel_size').value

        self.sub = self.create_subscription(
            PointCloud2, '/stereo/pointcloud', self.callback, 10
        )
        self.pub = self.create_publisher(
            PointCloud2, '/stereo/pointcloud_downsampled', 10
        )

        self.voxel_size = voxel_size

    def callback(self, msg):
        # Konvertiere zu NumPy
        points = np.array(list(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        )))

        # Erstelle Open3D PointCloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Voxel-Downsampling
        pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)

        # Zurück zu ROS 2
        points_down = np.asarray(pcd_down.points)

        self.get_logger().info(
            f'Downsampling: {len(points)} → {len(points_down)} Punkte '
            f'({100 * len(points_down) / len(points):.1f}%)',
            throttle_duration_sec=2.0
        )

        # Publiziere downsampled PointCloud2
        # ... (PointCloud2-Erstellung wie zuvor)
```

**Ergebnis**: Von 2 Mio. auf 50.000 Punkte → 40× schnellere Verarbeitung!

### 2. Rauschen und Ausreißer

**Problem**: Messfehler führen zu ungültigen Punkten

```ascii
Ideale Punktwolke:        Reale Punktwolke mit Rauschen:
      ████████                  ██ ████ ██
      ████████                  ███•██•███  ← Ausreißer
      ████████        vs.       ██ ████ ██
      ████████                  •████████•

Glatte Oberfläche         Rauschende Punkte + Ausreißer                                                     .
```

**Ursachen:**

+ Sensorrauschen (schlechte Beleuchtung, reflektierende Oberflächen)
+ Fehlende Disparität-Matches (texturlose Bereiche)
+ Rand-Effekte bei Stereo-Matching
+ Bewegungsunschärfe

**Lösungsansätze:**

**A) Statistischer Ausreißer-Filter**

Entfernt Punkte, die zu weit von ihren Nachbarn entfernt sind.

```python
# Open3D Statistischer Filter
pcd_filtered, ind = pcd.remove_statistical_outlier(
    nb_neighbors=20,      # Anzahl Nachbarn
    std_ratio=2.0         # Standardabweichungs-Faktor
)

# Punkte, die weiter als 2×σ vom Mittelwert ihrer 20 Nachbarn
# entfernt sind, werden entfernt
```

**B) Radius-Filter**

Entfernt Punkte mit zu wenigen Nachbarn in einem bestimmten Radius.

```python
# Open3D Radius-Filter
pcd_filtered, ind = pcd.remove_radius_outlier(
    nb_points=16,      # Mind. 16 Nachbarn erforderlich
    radius=0.05        # Innerhalb 5cm Radius
)
```

### 3. Fehlende Daten (Holes/Gaps)

**Problem**: Bereiche ohne gültige Tiefenwerte

```ascii
3D-Szene mit Lücken:
    ████████          ← Objekt erkannt
                      ← Lücke (keine Disparität)
       ?????          ← Texturlose Fläche (Wand)
                      ← Lücke (Okklusion)
    ████████          ← Boden erkannt                                                                   .
``` 

**Ursachen:**

+ **Texturlose Bereiche**: Glatte Wände, einfarbige Flächen → kein Matching möglich
+ **Okklusionen**: Objekt nur in einem Bild sichtbar
+ **Reflexionen**: Glänzende/spiegelnde Oberflächen
+ **Zu nah/zu fern**: Außerhalb des Disparitätsbereichs


**Lösungsansätze:**

| Ansatz | Beschreibung | Anwendung |
|--------|--------------|-----------|
| **Interpolation** | Lücken aus Nachbarn schätzen | Kleine Löcher |
| **Inpainting** | ML-basiertes Auffüllen | Größere Bereiche |
| **Multi-View** | Mehrere Kameraperspektiven | Okklusionen |
| **Strukturiertes Licht** | Projektor fügt Textur hinzu | Texturlose Flächen |

### 4. Koordinatensystem-Transformation

**Problem**: Punktwolke ist im Kamera-Koordinatensystem, nicht im Roboter-Koordinatensystem

```ascii
Kamera-Frame:              Roboter-Frame (base_link):
    Y                           X (vorwärts)
    ↑                           ↑
    |                           |
    └──→ X                      └──→ Y (links)
   ╱
  Z (vorwärts)              Z (nach oben)

Transformation erforderlich!                                   .
```

**Lösung**: TF2-Integration

```python
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class PointCloudTransformer(Node):
    def __init__(self):
        super().__init__('pointcloud_transformer')

        # TF2 Buffer und Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointCloud2, '/stereo/pointcloud', self.callback, 10
        )
        self.pub = self.create_publisher(
            PointCloud2, '/stereo/pointcloud_base', 10
        )

    def callback(self, msg):
        try:
            # Hole Transformation von camera_link → base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link',           # Ziel-Frame
                msg.header.frame_id,   # Quell-Frame (camera_link)
                rclpy.time.Time(),     # Aktuellste Transformation
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Transformiere Punktwolke
            cloud_transformed = do_transform_cloud(msg, transform)

            # Publiziere
            self.pub.publish(cloud_transformed)

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF-Fehler: {e}')
```

**Wichtig**: URDF/TF-Tree muss korrekt konfiguriert sein!

### 5. Zeitstempel-Synchronisation

**Problem**: Linkes und rechtes Bild kommen zu unterschiedlichen Zeiten an

```ascii
Zeit →
  t=0      t=10ms    t=20ms    t=30ms
Left:  ●─────────────●─────────────●
Right:     ●─────────────●─────────────●

❌ Falsch: Verwende Left(t=0) + Right(t=10)
✅ Richtig: Warte auf passende Paare                                                                 .
```

**Lösung**: `message_filters` für Synchronisation

```python
from message_filters import ApproximateTimeSynchronizer, Subscriber

class StereoSync(Node):
    def __init__(self):
        super().__init__('stereo_sync')

        # Message Filter Subscriber
        left_sub = Subscriber(self, Image, '/stereo/left/image_raw')
        right_sub = Subscriber(self, Image, '/stereo/right/image_raw')

        # Approximate Time Synchronizer
        # (erlaubt kleine Zeitunterschiede)
        self.sync = ApproximateTimeSynchronizer(
            [left_sub, right_sub],
            queue_size=10,
            slop=0.1  # Max. 100ms Zeitunterschied
        )

        self.sync.registerCallback(self.callback)

    def callback(self, left_msg, right_msg):
        # Beide Bilder haben (fast) denselben Zeitstempel!
        time_diff = abs(
            left_msg.header.stamp.sec - right_msg.header.stamp.sec +
            (left_msg.header.stamp.nanosec - right_msg.header.stamp.nanosec) * 1e-9
        )

        self.get_logger().info(
            f'Synchronisiert: Δt = {time_diff*1000:.1f}ms',
            throttle_duration_sec=2.0
        )

        # Verarbeite synchronisierte Bilder
        # ...
```

### 6. Bewegungsartefakte

**Problem**: Roboter/Kamera bewegt sich während der Aufnahme

```ascii
Statische Kamera:           Bewegte Kamera:
    ████████                    ████
    ████████                  ████  ████
    ████████        vs.         ████████  ← Verschmiert
    ████████                      ████

Klare Tiefenkarte           Verzerrte Tiefenkarte                                                  .
```

**Lösungen:**

+ **Höhere Bildrate**: Reduziert Motion Blur
+ **Shutter-Synchronisation**: Beide Kameras gleichzeitig auslösen
+ **IMU-Kompensation**: Bewegung herausrechnen
+ **Nur bei Stillstand messen**: Roboter stoppt für Messung


### Literatur und Ressourcen

**Bücher:**

+ Hartley & Zisserman: "Multiple View Geometry in Computer Vision"
+ Szeliski: "Computer Vision: Algorithms and Applications"

**Online-Ressourcen:**

+ OpenCV Documentation: https://docs.opencv.org/
+ ROS 2 sensor_msgs: https://docs.ros2.org/foxy/api/sensor_msgs/
+ Open3D Documentation: http://www.open3d.org/

**Tools:**

+ Kalibr: Kamera- und IMU-Kalibrierung (https://github.com/ethz-asl/kalibr)
+ Stereo Lab: Praktische Stereo-Tutorials
