# Übung 1 - Antworten-Vorlage

**Name**: [Ihr Name]
**Matrikelnummer**: [Ihre Matrikelnummer]
**Datum**: [Datum]

---

## Aufgabe 0.1: Bag-Datei inspizieren

### a) Anzahl GNSS-Nachrichten

**Antwort**:
- `/fixposition/gnss1`: _____ Nachrichten
- `/fixposition/gnss2`: _____ Nachrichten
- `/fixposition/odometry_llh`: _____ Nachrichten
- **Gesamt**: _____ Nachrichten

### b) Sensor mit höchster Nachrichtenrate

**Antwort**:
Topic: _______________
Anzahl Nachrichten: _____
Geschätzte Frequenz: _____ Hz (= Anzahl / Dauer)

### c) Vorteile des MCAP-Formats

**Antwort**:
[Ihre Antwort hier - ca. 3-5 Sätze]

Stichpunkte:
- Kompression
- Indexierung
- Kompatibilität
- ...

---

## Aufgabe 0.2: Visualisierung

### a) Beschreibung der Umgebung (PointCloud)

**Antwort**:
[Beschreiben Sie, was Sie in der PointCloud sehen]

Beobachtungen:
- Gebäude: ja/nein, welche Art?
- Straße/Weg: asphaltiert/unbefestigt?
- Vegetation: Bäume, Büsche?
- Andere Objekte: ...

### b) Farbcodierung der PointCloud

**Antwort**:
[Erklären Sie, welche Information durch die Farbe dargestellt wird]

Wenn "RGB8" als Color Transformer verwendet wird:
- Farben stammen von: _____________
- Vorteil: _____________

### c) Auswirkung des Fixed Frame

**Antwort**:
[Beschreiben Sie, was passiert, wenn Sie den Fixed Frame ändern]

Getestet:
- Fixed Frame = `base_link`: _____________
- Fixed Frame = `map`: _____________
- Fixed Frame = `zed2i_front_camera_link`: _____________

Unterschiede:
[Ihre Beobachtungen]

---

## Aufgabe 0.3: Topic-Analyse

### a) GNSS-Topic-Frequenzen

**Gemessene Frequenzen** (mit `ros2 topic hz`):

| Topic | Frequenz (Hz) |
|-------|---------------|
| `/fixposition/gnss1` | _____ |
| `/fixposition/gnss2` | _____ |
| `/fixposition/odometry_llh` | _____ |

### b) Message-Typ NavSatFix

**Antwort** (nutzen Sie `ros2 interface show sensor_msgs/msg/NavSatFix`):

Wichtige Felder:
- `latitude`: _____________
- `longitude`: _____________
- `altitude`: _____________
- `position_covariance`: _____________
- `status`: _____________

### c) QoS-Profile für GNSS-Topics

**Antwort** (aus `ros2 topic info /fixposition/gnss1 --verbose`):

- **Reliability**: _____________ (Reliable / Best Effort)
- **Durability**: _____________ (Volatile / Transient Local)
- **History**: _____________ (Keep Last / Keep All)
- **Depth**: _____________

**Bedeutung**:
[Kurze Erklärung, warum diese QoS-Settings für GNSS sinnvoll sind]

---

## Aufgabe 1.2: Trajektorien

### Gesamtstrecken

| GNSS-Quelle | Streckenlänge (m) |
|-------------|-------------------|
| GNSS1 | _____ |
| GNSS2 | _____ |
| Odometry LLH | _____ |

### Beobachtungen

**Unterschiede zwischen den Trajektorien**:
[Ihre Beobachtungen - wo weichen sie ab? Wo sind sie ähnlich?]

---

## Aufgabe 1.3: Drift-Analyse

### a) Sensor-Abweichungen

**Statistiken (GNSS1 vs. GNSS2)**:

- Mittlere Abweichung: _____ m
- Maximale Abweichung: _____ m
- Standardabweichung: _____ m

**Zeitpunkt maximaler Abweichung**: _____ Sekunden

**Mögliche Ursache**:
[Ihre Hypothese - Multipath? Abschattung? Gebäude?]

### b) Position Covariance

**Zeitbereiche mit hoher Unsicherheit**:
- Von _____ s bis _____ s: σ_lat ≈ _____ m
- Von _____ s bis _____ s: σ_lon ≈ _____ m

**Korrelation mit Umgebung**:
[Schauen Sie sich das Video an - was passiert zu diesen Zeitpunkten?]
- Gebäude in der Nähe?
- Bäume/Vegetation?
- Tunnel/Unterführung?

---

## Aufgabe 1.4: Fusion-Vergleich (Bonus)

### Glattheit der Trajektorie

**Metriken**:
- Summe der Beschleunigungen (GNSS1): _____
- Summe der Beschleunigungen (Fusion): _____

**Interpretation**:
[Welche Trajektorie ist glatter? Warum ist das wichtig?]

### Rausch-Charakteristik

**Beobachtungen**:
[Hochfrequenz-Komponenten? Ausreißer?]

### Vertrauen in fusionierte Lösung

**Ihre Einschätzung**:
[Würden Sie der fusionierten Lösung vertrauen? Warum/warum nicht?]

Argumente:
- Pro: _____________
- Contra: _____________

---

## Reflexion

### Was haben Sie gelernt?

[3-5 Sätze über die wichtigsten Erkenntnisse]

### Schwierigkeiten

[Welche Probleme hatten Sie? Wie haben Sie sie gelöst?]

### Zeitaufwand

Gesamtzeit: _____ Stunden

Verteilung:
- Teil A (ROS2-Tools): _____ min
- Teil B (Programmierung): _____ min

---

**Abgabe**: Diese Datei zusammen mit allen anderen Ergebnissen als Archiv einreichen.
