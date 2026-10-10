# Kurslandkarte

Interaktive Übersicht über Begriffe, Zusammenhänge und didaktische Fäden der Veranstaltung Robotik für Studierende — über Vorlesungen (VL), Übungen (Ü) und Praktika (P) in WiSe 2026/27 und SoSe 2027. Drei Ansichten: Termine (Liste, Standard auf dem Handy), Metrokarte und Zeitleiste. „Sie sind hier“ ergibt sich aus dem Datum. Veröffentlicht über GitHub Pages aus dem Ordner `docs/`.

Die Ansicht stammt aus der Kurslandkarte der Vorlesung EAVD (Projekt invivis) und ist hier so verallgemeinert, dass Termintypen, Blöcke und Themengebiete aus den Daten kommen.

| Datei | Inhalt |
| :---- | :----- |
| `erzeuge.py` | Kuratierte Daten (Termine, Begriffe, Beziehungen, Fäden) und Erzeugung von Daten und Seite |
| `vorlage.html` | Ansicht; `%%DATEN%%` und `%%STAND%%` werden beim Erzeugen ersetzt |
| `../docs/kurslandkarte/kurslandkarte.json` | erzeugt: alle Daten, u. a. für das Projekt invivis |
| `../docs/kurslandkarte/index.html` | erzeugt: die veröffentlichte Seite |

## Aktualisieren

Nach Änderungen an Materialien, Terminen oder Begriffen im Wurzelverzeichnis des Repositorys:

```bash
python3 kurslandkarte/erzeuge.py
```

Begriffe verweisen in `erzeuge.py` auf den **Titel** ihres Abschnitts (ohne Backticks); das Skript liest die Abschnitte automatisch aus den Materialdateien und ermittelt daraus die LiaScript-Foliennummer. Das Skript bricht ab, wenn ein Abschnittstitel nicht mehr existiert, eine Beziehung auf einen unbekannten Begriff verweist, ein Faden nicht chronologisch verläuft oder die Termine nicht nach Datum sortiert sind. Die erzeugten Dateien in `docs/` werden mit eingecheckt.

**Wird ein Kapitel überarbeitet** (z. B. ROS-1-Abschnitte entfernt), meldet das Skript verschwundene Abschnittstitel — dann den Begriff in `erzeuge.py` auf den neuen Titel umstellen und den Status des Termins in `T` von `"Bestand"` auf `"ausgearbeitet"` setzen.

## Datenmodell

* **Kurs:** `basis` (LiaScript-Link-Präfix), `speicher` (Schlüssel für Einstellungen im Browser)
* **Termin** (`vorlesungen`): `vl` (Position), `key`, `label` (z. B. „VL 03“, „Ü 1“, „P 2“), `datei` (oder `null`, solange kein Material existiert), `datum` (Montag der KW nach `Zeitplan.ods`, ISO), `kurz`, `status` (`ausgearbeitet` | `Bestand` | `geplant`), `block`, `abschnitte`
* **Phase:** `von`, `bis` (Terminpositionen), `name` (Block), `farbe`
* **Linie** (Themengebiet): `id`, `name`; die Farben stehen als `--l-{id}` in `vorlage.html`
* **Begriff:** `id`, `name`, `linie`, `vl` (Terminposition), `abschnitt` (Foliennummer), `ebene` (1 = Kernbegriff, 2 = Detailbegriff), `code`, `erklaerung`, `fehlvorstellung`
* **Beziehung:** `von`, `nach`, `text` (gelesen als „von … text … nach“)
* **Faden:** `id`, `name`, `beschreibung`, `stationen` (chronologische Folge von Begriffs-ids, höchstens sechs Fäden)

## Offen

* Die Termine folgen dem vorläufigen Zeitplan (Ansatz 1); die Zuordnung zu den Studiengängen ROB/BAI steht noch aus.
* Für Übungen und Praktika ohne Material sowie die neuen Vorlesungen (LiDAR, Punktwolken, SLAM) gibt es noch keine Begriffe.
