# Kurslandkarte

_Autoren: Sebastian Zug & Claude Code (Opus 5.5) mit LiaScript Teaching-Agent_

Interaktive Übersicht über Begriffe, Zusammenhänge und didaktische Fäden der Veranstaltung Robotik für Studierende — über Vorlesungen (VL), Übungen (Ü) und Praktika (P) in WiSe 2026/27 und SoSe 2027. Drei Ansichten: Termine (Wochenraster nach Blöcken mit Spalten VL/Ü/P, Standard auf dem Handy), Metrokarte und Zeitleiste. „Sie sind hier“ ergibt sich aus dem Datum. Veröffentlicht über GitHub Pages aus dem Ordner `docs/`.

Karte, Startseite und Orientierungsfolien erzeugt der gemeinsame Kern invivis Kurslandkarte (Projekt invivis), den auch EAVD und Eingebettete Systeme nutzen; Termintypen, Blöcke und Themengebiete kommen aus den Daten.

| Datei | Inhalt |
| :---- | :----- |
| `erzeuge.py` | Kuratierte Kursdaten (Termine, Begriffe, Beziehungen, Fäden) und Aufruf des Kerns |
| `invivis/` | Kopie des gemeinsamen Kerns [invivis Kurslandkarte](https://github.com/TUBAF-IfI-LiaScript/invivis-kurslandkarte) (`kern.py`, `vorlage.html`, `startseite.html`) – **nicht hier ändern**, Verbesserungen im Kern-Repository machen und mit `uebernehmen.py` verteilen |
| `../docs/index.html` | erzeugt: Startseite des Kurses („Diese Woche“, Kurslandkarte, Lernziele, Organisation, Kontakt) |
| `../docs/kurslandkarte/kurslandkarte.json` | erzeugt: alle Daten, u. a. für das Projekt invivis |
| `../docs/kurslandkarte/index.html` | erzeugt: die veröffentlichte Seite |

## Aktualisieren

Nach Änderungen an Materialien, Terminen oder Begriffen im Wurzelverzeichnis des Repositorys:

```bash
python3 kurslandkarte/erzeuge.py          # Karte und Startseite
python3 kurslandkarte/erzeuge.py --pdf    # vorher fehlende/veraltete PDFs nach `docs/pdf/` (liaex)
```

Kacheln und Startseite verlinken die PDFs aus `docs/pdf/`; sie werden mit eingecheckt.

Begriffe verweisen in `erzeuge.py` auf den **Titel** ihres Abschnitts (ohne Backticks); das Skript liest die Abschnitte automatisch aus den Materialdateien und ermittelt daraus die LiaScript-Foliennummer. Das Skript bricht ab, wenn ein Abschnittstitel nicht mehr existiert, eine Beziehung auf einen unbekannten Begriff verweist, ein Faden nicht chronologisch verläuft oder die Termine nicht nach Datum sortiert sind. Die erzeugten Dateien in `docs/` werden mit eingecheckt.

**Wird ein Kapitel überarbeitet** (z. B. ROS-1-Abschnitte entfernt), meldet das Skript verschwundene Abschnittstitel — dann den Begriff in `erzeuge.py` auf den neuen Titel umstellen und den Status des Termins in `T` von `"Bestand"` auf `"ausgearbeitet"` setzen.

## Nach jedem Termin: Begriffe freischalten

Die Karte zeigt die Begriffe freigegebener Termine; die übrigen erscheinen mit „Ausblick: An“ blass. Ein Direktlink aus den Folien auf einen noch nicht freigegebenen Begriff schaltet den Ausblick automatisch ein. Nach einem Termin: Begriffe mit dem Behandelten abgleichen, den Termin-key in `FREIGEGEBEN` eintragen, `python3 kurslandkarte/erzeuge.py`, committen.

## Datenmodell

* **Kurs:** `basis` (LiaScript-Link-Präfix), `speicher` (Schlüssel für Einstellungen im Browser)
* **Termin** (`vorlesungen`): `vl` (Position), `key`, `label` (z. B. „VL 2“, „Ü 1“, „P 2“; Vorlesungen werden in `erzeuge.py` automatisch laufend nach Zeitplan nummeriert — die Ordnernummern im Repo stammen aus der alten Reihenfolge), `datei` (oder `null`, solange kein Material existiert), `datum` (Montag der KW nach `Zeitplan.ods`, ISO), `kurz`, `status` (`ausgearbeitet` | `Bestand` | `geplant`), `block`, `abschnitte`
* **Phase:** `von`, `bis` (Terminpositionen), `name` (Block), `farbe`
* **Linie** (Themengebiet): `id`, `name`, `farbe` (Palette `c1`…`c7` des Kerns)
* **Begriff:** `id`, `name`, `linie`, `vl` (Terminposition), `abschnitt` (Foliennummer), `ebene` (1 = Kernbegriff, 2 = Detailbegriff), `code`, `erklaerung`, `fehlvorstellung`
* **Beziehung:** `von`, `nach`, `text` (gelesen als „von … text … nach“)
* **Faden:** `id`, `name`, `beschreibung`, `stationen` (chronologische Folge von Begriffs-ids, höchstens sechs Fäden)

## Offen

* Die Termine folgen dem vorläufigen Zeitplan (Ansatz 1); die Zuordnung zu den Studiengängen ROB/BAI steht noch aus.
* Für Übungen und Praktika ohne Material sowie die neuen Vorlesungen (LiDAR, Punktwolken, SLAM) gibt es noch keine Begriffe.
