<!--
author: Johannes Kohl, Institut für Informatik, TU Bergakademie Freiberg
email: johannes-georg.kohl@informatik.tu-freiberg.de
version: 1.0.0
language: de
narrator: Deutsch Female

comment: Praktikum 1 des Robotik-Projekts. Inhalt: Gruppe, gemeinsamer Workspace mit Repository sowie Planung und Aufnahme von ROS2-Bags.
-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/exercises/praktikum_01_ros2_bags/README.md)

# Praktikum 1: Organisation und Bag-Aufnahme

--{{0}}--
Willkommen zum ersten Praktikum. Heute schaffen wir die organisatorische Basis für Ihr Projekt und überlegen gemeinsam, welche Daten Sie für die späteren Praktika aufnehmen müssen.

| Teil | Inhalt | Dauer |
|:-----|:-------|:------|
| Teil 1 | Gruppe, Repository, Workspace | ≈ 30 min |
| Teil 2 | Planung: Aufgabe, Denkanstöße, Querschnitt | ≈ 20 min |
| Teil 2 | Kurzvorstellung der Ideen | ≈ 15 min |
| Teil 2 | Gemeinsame Diskussion | ≈ 15 min |
| Teil 2 | Reihenfolge und Aufnahme | verbleibende Zeit |

Für das Praktikum stehen **zwei Termine à 90 Minuten** zur Verfügung; der Raum kann darüber hinaus jederzeit für weitere Aufnahmen frei genutzt werden.

     {{1}}
************************************

**Ziel des Tages:**

- Jedes Team hat **ein** gemeinsames Repository mit einem Paketordner für alle eigenen Pakete.
- Alle Teammitglieder arbeiten vom **gleichen Stand** aus.
- Jedes Team hat erste Aufnahmen im Kasten und weiß, **welche Daten** es für die späteren Praktika braucht.

************************************

<!-- kurslandkarte:wo-start -->

## Wo stehen wir?

**Ziel im Projekt:** Ihr TurtleBot ist verbunden, fährt sicher, und Sie haben eine Testfahrt aufgezeichnet – das Rohmaterial für Block 2.

**Block 1 · ROS-Grundlagen** – Termin 7 von 32, heute farbig markiert

<!-- data-type="none" -->
| Woche | Vorlesung | Übung | Praktikum |
| :-- | :-- | :-- | :-- |
| KW 43 · 19.–23.10. | [VL 00 Einführung](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/master/00_Einfuehrung/00_Einfuehrung.md) · Mi 21.10. |  |  |
| KW 44 · 26.–30.10. | [VL 01 ROS 2 Grundlagen](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/master/01_ROS_Grundlagen/01_ROS_Grundlagen.md) · Mi 28.10. | [Ü 1 Linux Shell](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/master/exercises/uebung_01_linux_shell/README.md) · Do 29.10. |  |
| KW 45 · 2.–6.11. | [VL 02 ROS 2 Pakete](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/master/02_ROS_Pakete/02_ROS_Pakete.md) · Mi 4.11. |  |  |
| KW 46 · 9.–13.11. | [VL 03 Koordinaten + QoS](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/master/03_Koordinaten_QoS/03_Koordinaten_QoS.md) · Do 12.11. | [Ü 2 Erster Workspace](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/master/exercises/uebung_02_workspace/README.md) · Do 12.11. |  |
| **KW 48 · 23.–27.11.**<!-- style="background-color: rgba(255, 166, 0, 0.35); padding: 2px 6px; border-radius: 4px;" --> |  |  | **[P 1 TurtleBot + Bags](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/master/exercises/praktikum_01_ros2_bags/README.md)**<!-- style="background-color: rgba(255, 166, 0, 0.35); padding: 2px 6px; border-radius: 4px;" --> · Mi 25.11. + Do 26.11. |

- **Sie brauchen:** [Topics des TurtleBot](https://tubaf-ifi-liascript.github.io/VL_SoftwareprojektRobotik/kurslandkarte/#begriff=tbtopics) (VL 01) · [ros2 topic …](https://tubaf-ifi-liascript.github.io/VL_SoftwareprojektRobotik/kurslandkarte/#begriff=topiccli) (VL 01) · [ROS\_DOMAIN\_ID](https://tubaf-ifi-liascript.github.io/VL_SoftwareprojektRobotik/kurslandkarte/#begriff=domainid) (Ü 1) · [QoS-Verträglichkeit](https://tubaf-ifi-liascript.github.io/VL_SoftwareprojektRobotik/kurslandkarte/#begriff=qoscompat) (VL 03) · [tf2](https://tubaf-ifi-liascript.github.io/VL_SoftwareprojektRobotik/kurslandkarte/#begriff=tf2) (VL 03) · [tf-Baum](https://tubaf-ifi-liascript.github.io/VL_SoftwareprojektRobotik/kurslandkarte/#begriff=tftree) (VL 03) · [Twist / cmd\_vel](https://tubaf-ifi-liascript.github.io/VL_SoftwareprojektRobotik/kurslandkarte/#begriff=twist) (Ü 2) · [Parameter und Parameterdatei](https://tubaf-ifi-liascript.github.io/VL_SoftwareprojektRobotik/kurslandkarte/#begriff=paramfile) (Ü 2) · [ros2 bag](https://tubaf-ifi-liascript.github.io/VL_SoftwareprojektRobotik/kurslandkarte/#begriff=rosbag) (Ü 2)
- **Heute:** anwenden, nichts Neues – Ihre Arbeit hier ist die Prüfungsleistung

> [!TIP]
> **Tipp – Kurslandkarte:** [Ganze Kurslandkarte öffnen](https://tubaf-ifi-liascript.github.io/VL_SoftwareprojektRobotik/kurslandkarte/#termin=p1) – dort finden Sie jeden Begriff mit Erklärung, typischer Stolperfalle, Link auf die passende Folie und die roten Fäden durch die Veranstaltung.

<!-- kurslandkarte:wo-ende -->

## Teil 1: Organisation

--{{0}}--
Der erste Teil ist kurz, Sie verteilen die Teamrollen und legen eine gemeinsame Basis für alle weiteren Praktika.

In der Übung haben Sie, jeder für sich, ein eigenes Paket gebaut. Jetzt machen wir dasselbe **gemeinsam** und **versioniert**.

     {{1}}
************************************

**Ablauf in vier Schritten:**

1. Finden Sie sich in ihren Gruppen zusammen und entscheiden Sie, **wer** das Repository anlegt.
2. Diese Person legt einen **ROS2-Workspace** mit einem Paketordner an, z.B. `ros_ws/src/turtlebot3_praktikum_pakete`.
3. Während dessen wird in der [GitHub Organisation](https://github.com/orgs/IfI-TUBAF-Robotik-2026/people/pending_invitations) ein Repository angelegt.
4. Danach wird im Paketordner ein lokales Repository angelegt und auf das remote Repository gepushed.

************************************

     {{2}}
> **Warum ein Repository pro Team?**
>
> Ab dem nächsten Praktikum schreiben Sie Code für Bildkorrektur, Spurerkennung und Sensorfusion. Ohne gemeinsamen Stand tauschen Sie am Ende Zip-Dateien aus und wissen nicht mehr, welche Version tatsächlich läuft.

### Schritt 1: Gruppe und Verantwortlichkeit

Wählen Sie eine Person aus Ihrem Team, welche sich initial um das Setup des Repositorys kümmert.

Am besten übernimmt dieselbe Person auch die Organisation innerhalb der Gruppe über den gesamten Verlauf des Kurses.
Alternativ können Sie die Gruppenverantwortlichkeit auch zu gegebenem Zeitpunkt wechseln lassen.
Entscheiden Sie dies bitte **vorher** untereinander und halten sie entsprechende Absprachen schriftlich fest um Missverständnisse zu vermeiden.

Gute Kriterien:
- [ ] heute bis zum Ende anwesend
- [ ] kann Strukturfragen im Team entscheiden
- [ ] GitHub-Zugang funktioniert (SSH-Key oder Personal Access Token)

### Schritt 2: Workspace und Paketordner anlegen

Die **Repository-Verantwortlichen** arbeiten jetzt vor, alle anderen können entweder kurz abwarten, das zugehörige Repository in GitHub anlegen oder schonmal Teil 2 vorarbeiten.

     {{0}}
************************************

```bash
mkdir -p ros_ws/src/turtlebot3_praktikum_pakete
cd ros_ws
colcon build
```

> **Was passiert hier?**
>
> - `mkdir -p` legt fehlende Elternverzeichnisse gleich mit an. Ein einziger Aufruf genügt daher für den gesamten Pfad aus `ros_ws`, `src/` und Paketordner.
> - `src/` ist der Ordner, in dem `colcon` nach Paketen sucht.
> - `src/turtlebot3_praktikum_pakete/` ist ein **Sammelordner** für alle Pakete Ihres Teams. Er ist selbst **kein** ROS2-Paket, sondern nimmt in den nächsten Praktika mehrere davon auf.
> - Mit `colcon` können Sie den noch leeren Workspace bauen, wobei `build/`, `install/` und `log/` entstehen.

************************************

     {{1}}
************************************

Warum ein Sammelordner statt einem großen Paket?

- **Ein ROS2-Paket soll genau einen Zweck erfüllen.** Bildkorrektur, Spurerkennung und Sensorfusion werden deshalb getrennte Pakete.
- `colcon` durchsucht `src/` **rekursiv** und findet daher Pakete in Unterordnern genauso wie solche direkt in `src/`.
- **Das Repository ist der Paketordner, nicht der Workspace.** So landen `build/`, `install/` und `log/` nie im Repository, und ein `git clone` liefert direkt alle Pakete des Teams.

> **Später:** Wenn Sie ein neues Paket brauchen, wechseln Sie mit `cd ros_ws/src/turtlebot3_praktikum_pakete` in diesen Ordner und legen es dort mit `ros2 pkg create ...` an.

************************************

     {{2}}
************************************
So sieht der Workspace danach aus:

```txt
ros_ws/
├── src/
│   └── turtlebot3_praktikum_pakete/   ← wird das Repository
├── build/
├── install/
└── log/
```

> **Hinweis:** Der Ordnername `turtlebot3_praktikum_pakete` ist nur ein Beispiel. Sie dürfen ihn beliebig benennen, vermeiden Sie aber Bindestriche (`-`) und verwenden Sie stattdessen Underscores (`_`). Soll der Ordner beim Klonen nicht wie das Repository heißen, geben Sie den gewünschten Namen als zweites Argument an: `git clone <url> <ordnername>`.

> **Hinweis:** In der Regel muss nach jedem `colcon build` in jeder betroffenen Shell einmal `source install/setup.bash` ausgeführt werden. Im noch leeren Workspace bewirkt das nichts, das Muster bleibt jedoch gleich.

************************************

     {{3}}
************************************

Welcher Befehl baut den Workspace?

[( )] `ros2 build`
[(X)] `colcon build`
[( )] `colcon make`
[( )] `ros2 pkg build --workspace`
[[?]] Der Befehl stammt nicht aus dem ROS2-CLI, sondern aus dem Build-Werkzeug für ROS-Workspaces.

************************************

### Schritt 3: Remote-Repository anlegen

Sie können das Repository anlegen, während der Workspace entsteht. Wichtig ist nur, dass es vor dem ersten Push existiert.

1. Prüfen Sie, ob Sie in der Organisation [IfI-TUBAF-Robotik-2026](https://github.com/IfI-TUBAF-Robotik-2026) eingetragen sind. Offene Einladungen sehen Sie im Zweifel [hier](https://github.com/notifications).
2. Legen Sie in der Organisation ein neues Repository an: [github.com/organizations/IfI-TUBAF-Robotik-2026/repositories/new](https://github.com/organizations/IfI-TUBAF-Robotik-2026/repositories/new)
3. **Name:** nach Belieben, fühere Gruppen haben die Namen der Gruppenmitglieder verwendet (z.B. LaMaJaTa).
4. **Ohne** README, **ohne** `.gitignore` und **ohne** Lizenz, denn diese Dateien legen Sie lokal an. Sonst gibt es beim ersten Push einen Konflikt.
5. Zuletzt `Create repository`. Die angezeigten Befehle brauchen Sie gleich, führen Sie sie aber **im Paketordner** aus und nicht in einer höheren Ebene (z.B. `src/` oder Workspace).

### Schritt 4: Dateien und lokales Repository

Im Paketordner legen Sie zunächst nur zwei Dateien an, weil die eigentlichen Pakete erst in den nächsten Praktika entstehen. Beide sind trotzdem nötig, denn **Git versioniert keine leeren Ordner**: Ohne Inhalt gäbe es nichts zu committen.

     {{1}}
************************************

**README.md**

Legen Sie im Paketordner eine `README.md` an.
Darin notieren Sie ihr Team, wichtige Befehle und eine Übersicht der Pakete, z.B.:

```md
# Praktikumspakete Team P1

## Team

| Name             | MNr   | Nutzername       |
|:-----------------|:------|:-----------------|
| Erika Mustermann | 12345 | @erikamustermann |

## Bauen und starten

~~~bash
cd ros_ws
colcon build
source install/setup.bash
~~~

## Pakete
| Paket | Zweck |
|:------|:------|
| ...   | ...   |
```

************************************

     {{2}}
************************************

**.gitignore**

Erstellen Sie eine .gitignore Datei mit folgendem Inhalt:

```gitignore
__pycache__/
*.pyc

# um versehentliche Uploads zu vermeiden:
**/*.mcap
**/*.db3
**/build/
**/install/
**/log/
```

> **Was steht hier drin?**
>
> - `build/`, `install/` und `log/` liegen im Workspace **außerhalb** des Repositories, dafür braucht die `.gitignore` eigentlich keinen Eintrag. Um sicherzustellen, dass Sie diese Ordner nicht versehentlich mit hochladen, können Sie die entsprechenden Einträge übernehmen.
> - `*.mcap` (und `*.db3` bei `--storage sqlite3`) sind die eigentlichen Bag-Daten und werden schnell mehrere GB groß. GitHub LFS wäre zwar eine Option, für große Bags empfehlen wir aber den **manuellen Transfer**, z. B. per USB-Stick.
> - `__pycache__/` und `*.pyc` entstehen beim Ausführen von Python und gehören nicht ins Repository.

************************************

     {{3}}
************************************

**Lokales Repository**

```bash
cd ros_ws/src/turtlebot3_praktikum_pakete

git init -b main
git add .
git status          # Kontrolle: nur README.md und .gitignore
git commit -m "Initiale Struktur für die Praktikumspakete"
```

> **Was passiert hier?**
>
> - `git init -b main` legt einen Branch `main` an; bei älteren Git-Versionen geht das mit `git branch -M main`.
> - `git status` zeigt vor dem Commit, was wirklich aufgenommen wurde.

************************************

     {{4}}
************************************

**Was gehört ins Repository?**

[[X]] `README.md` mit Team und Befehlen
[[X]] `.gitignore`
[[ ]] `ros_ws/build/`
[[ ]] `ros_ws/install/setup.bash`
[[ ]] `ros_ws/src/turtlebot3_praktikum_pakete/__pycache__/`

************************************

### Push ins Remote-Repository

Jetzt wird das lokale Repository mit dem Remote-Repository aus Schritt 3 verknüpft und hochgeladen:

```bash
cd ros_ws/src/turtlebot3_praktikum_pakete
git remote add origin <ssh oder https git url ihres Repositories>
git push -u origin main
```

> **Was passiert hier?**
>
> - `git remote add origin ...` hinterlegt die Adresse des Remote-Repositories; `origin` ist nur der Name dieser Verknüpfung.
> - `git push -u origin main` lädt den Branch `main` hoch und merkt sich die Verknüpfung. Später genügt `git push`.
> - Laden Sie danach die Repository-Seite im Browser neu: `README.md` und `.gitignore` sollten dort auftauchen.

> **Hilfe, das geht nicht!**
>
> - `Permission denied (publickey)` → für Ihr Konto ist kein SSH-Key hinterlegt. Legen Sie einen Key an und tragen Sie ihn bei GitHub ein, oder verwenden Sie HTTPS mit Personal Access Token.
> - `Repository not found` → Tippfehler im Namen, oder Sie sind noch nicht Mitglied der Organisation.
> - Es wird nach Benutzername und Passwort gefragt → das Passwort funktioniert nicht; nutzen Sie ein Token (HTTPS) oder SSH.
> - `src refspec main does not match any` → es gibt noch keinen Commit, oder der Branch heißt anders (`git branch -M main`).
> - `remote origin already exists` → die Verknüpfung existiert schon. Prüfen Sie ob sie korrekt ist und korrigieren Sie sie gegebenenfalls mit `git remote set-url origin <neue-url>`.

### Gemeinsamer Startpunkt

     {{0}}
************************************

Alle anderen im Team holen sich jetzt genau diesen Stand:

```bash
mkdir -p ros_ws/src/
cd ros_ws/src

# Eine der beiden Varianten wählen:

# Ordner soll wie das Repository heißen:
git clone <ssh oder https git url ihres Repositories>

# Eigener Name (zweites Argument), z.B. der Ihres Paketordners:
git clone <ssh oder https git url ihres Repositories> turtlebot3_praktikum_pakete

cd ..
colcon build
source install/setup.bash
```

> **Was passiert hier?**
>
> - Geklont wird **nach `src/`**, nicht in den Workspace. Nur dort sucht `colcon` nach Paketen.
> - `git clone` legt den Ordner nach dem Repository-Namen an. Soll er anders heißen, geben Sie den gewünschten Namen als zweites Argument an: `git clone <url> <ordnername>`.
> - Im Ordner liegt noch kein Paket, deshalb meldet `colcon build` nur `Summary: 0 packages finished`. Das ist an dieser Stelle richtig so.
> - `build/`, `install/` und `log/` entstehen bei jeder Person lokal neu und sind nicht Teil des Repositories.

************************************

     {{1}}
************************************

**Ab jetzt gilt für jede Änderung derselbe Ablauf:**

```bash
cd ros_ws/src/...   # in den Paketordner (= das Repository)
git pull            # erst holen, was die anderen gemacht haben

# ... arbeiten ...

cd ../..            # zurück in den Workspace
colcon build        # bauen und testen
source install/setup.bash

cd src/...          # wieder in den Paketordner
git status          # prüfen, was in den Commit soll
git add .
git commit -m "Was geändert wurde und warum"
git push
```

> **Tipp:** Bauen und starten Sie vor dem Push. Sonst merkt erst die nächste Person, dass etwas nicht kompiliert. Wenn Sie parallel an mehreren Features arbeiten, sprechen Sie sich ab und nutzen Branches, statt gleichzeitig auf `main` zu committen.

************************************

### Checkpoint Teil 1

Bevor es mit den Bags weitergeht, hakt **jedes** Team diese Punkte ab:

- [ ] Alle Teammitglieder sind in der GitHub-Organisation eingetragen.
- [ ] Das Remote-Repository existiert in der Organisation und ist Ihrem Team zuzuordnen (Name und/oder README).
- [ ] Der Paketordner liegt in `ros_ws/src/` und enthält `README.md` und `.gitignore`.
- [ ] Jedes Teammitglied hat geklont, und `colcon build` läuft ohne Fehler durch.
- [ ] `README.md` nennt Teammitglieder, die Befehle zum Bauen und Starten und dokumentiert die Pakete.

> **Warum der Aufwand?** Ab dem zweiten Praktikum arbeiten Sie an Sensorik und Bildverarbeitung. Wer dann noch unterschiedliche Stände zusammenführen muss, verliert Zeit, die für die eigentliche Aufgabe fehlt.

## Teil 2: Bag-Aufnahme

In den nächsten Praktika bearbeiten Sie:

- **Bildkorrektur** von Farbstich und Fisheye-Verzerrung,
- **Linien- und Spurerkennung** inklusive Haltelinien,
- **Hindernis- und Schildererkennung**,
- **Sensorfusion** von Kamera, LiDAR, IMU und Odometrie,
- **Regelung und Navigation** auf einer Strecke mit Parkbuchten und Kreuzung.

Dafür brauchen Sie Daten, die Sie vom Roboter erhalten. Aber ist es wirklich sinnvoll, für jeden Test vor Ort sein zu müssen?

**Ziel dieses Teils:** Jedes Team hat am Ende mindestens einen Bag, mit dem spätere Aufgaben **ohne Roboter** getestet werden können.

> **Hinweis:** Jeder Roboter sieht anders aus, jede Kamera hat einen anderen Farbstich, jeder LiDAR rauscht anders. Versteifen Sie sich also auch nicht zu sehr auf Ihr jeweiliges Bagfile!

### Aufgabe: Planen, was aufgenommen wird

--{{0}}--
Jetzt sind Sie dran: zehn bis fünfzehn Minuten Planung im Team. Danach stellt jede Gruppe ihre Ideen in zwei Minuten vor.

Beantworten Sie für Ihr Team die folgenden vier Fragen. Stichpunkte genügen:

1. **Welche Szenarien** brauchen wir? (Gerade, Kurven, Kreuzung, Parkbuchten, Hindernisse, Schilder, Personen, …)
2. **Wann muss was aufgenommen werden?** Reicht eine lange Fahrt mit allem, oder braucht es mehrere kurze Aufnahmen?
3. **Welche Perspektiven und Edge Cases** gibt es? (Abstand, Winkel, Beleuchtung, Reflexionen, Verschmutzung, Verdeckung, …)
4. **Wie speichern und verteilen** wir die Bags? (GitHub LFS, USB-Stick, Benennung, Zugriff für alle?)

**Ergebnis:** eine Liste „Szenario → Zweck → benötigte Topics → geschätzte Dauer“, zum Beispiel:

| Szenario | Zweck (Praktikum) | Topics | geschätzte Dauer |
|:---------|:------------------|:-------|:-----------------|
| Gerade mit Haltelinie | Spurerkennung (P2) | Kamera, `/scan` | 30 s |
| Kurve mit Begrenzungslinien | Krümmung (P2, P4) | Kamera, `/imu`, `/odom`, `/tf` | 60 s |
| … | … | … | … |

> **Denken Sie an die Zeit:** Sie können heute nicht beliebig viel aufnehmen. Priorisieren Sie Szenarien, die sich später **nicht** mehr herstellen lassen, weil der Roboter dann belegt ist oder der Aufbau zu aufwändig wäre. Natürlich können Sie aber auch außerhalb des Vorlesungs-/Übungszeitslot Aufnahmen vor Ort tätigen.

### Denkanstöße: Sensorik und Bildverarbeitung

Die folgenden Fragen stammen aus den Praktikumsinhalten der späteren Blöcke. Sie sind **Denkrichtungen, keine Pflichtliste**. Was davon für Ihre Aufnahmen relevant ist, entscheiden Sie.

     {{1}}
************************************

**Bildkorrektur und Spurerkennung**

- **Farbstich:** dieselbe Szene mit verschiedenen Kameras und bei unterschiedlicher Beleuchtung
- **Fisheye-Verzerrung:** Muster mit geraden Linien (Schachbrett, Fliesen, Latten), aus denen sich später entzerren und kalibrieren lässt
- **Haltelinien und Begrenzungslinien** aus verschiedenen Abständen, Winkeln und Krümmungen
- **Lichtreflexionen und Schmutz:** Was macht die Erkennung, wenn der Boden spiegelt? Woran erkennt man den Unterschied zu einem Hindernis?

************************************

     {{2}}
************************************

**LiDAR**

- **Winkelauflösung:** wie verändert sich die Genauigkeit, wenn man langsam auf ein weit entferntes Objekt zufährt?
- **Minimaler und maximaler Abstand:** bei welchen Entfernungen sind Objekte zu erkennen? Bei welchen nicht?

************************************

### Denkanstöße: Erkennung, Fusion, Navigation

Die folgenden Fragen stammen aus den Praktikumsinhalten der späteren Blöcke. Sie sind **Denkrichtungen, keine Pflichtliste**. Was davon für Ihre Aufnahmen relevant ist, entscheiden Sie.

     {{1}}
************************************

**Objekterkennung**

- **Höhe der Hindernisse:** welche Boxen liegen auf LiDAR-Ebene, welche sieht nur die Kamera? Bliebe dieser Unterschied in einem Bag erkennbar?
- **Schilder:** aus welchen Winkeln und Entfernungen sind Wenden, Links, Rechts, Gerade und Parken noch lesbar? Und was macht wechselndes Licht daraus?

************************************

     {{2}}
************************************

**Sensorfusion**

- **Kurvenradien:** wie gut folgt die Odometrie einer gleichmäßigen Kurve, wenn die Kamera die Krümmung liefert? Driften beide Messungen zeitlich auseinander?
- **Bezug der Sensoren:** ließe sich ohne `/tf` und `/tf_static` noch sagen, wo ein Messwert relativ zum Roboter entstanden ist?

************************************

     {{3}}
************************************

**Regelung und Navigation**

- **Streckenverlauf:** welche Teile sieht die Kamera aus der späteren Fahrtperspektive? Kommen Haltelinien, zwei Hindernisse und Parkbuchten darin vor?
- **Parken:** belegte und freie Buchten, verschiedene Anfahrtswinkel, 3 s Wartezeit im Parkmodus. Welche dieser Zustände müsste eine Aufnahme zeigen?
- **Kreuzung:** liegt die Zufahrt samt Haltelinie und abbiegenden Straßen im Sichtfeld? Oder bräuchte es dafür eine andere Blickrichtung?

************************************

### Querschnitt: Was fast immer dazugehört

Unabhängig vom Szenario lohnt es sich, über die folgenden Punkte vorher nachzudenken, weil sie erfahrungsgemäß vergessen werden:

- **Alle Sensoren gleichzeitig:** würden eine Kamera- und eine LiDAR-Aufnahme, getrennt aufgenommen, für eine Fusion überhaupt noch zusammenfinden?
- **Ruhephasen am Anfang und Ende:** wie lassen sich Offset, Drift und Ruhelage der IMU bestimmen, wenn die Aufnahme schon während der Fahrt beginnt?
- **Gemeinsame Uhr:** decken in `ros2 bag info` alle Topics denselben Zeitraum ab? Und was macht `use_sim_time` beim Abspielen ohne `--clock`?
- **Konstante Bedingungen:** ließe sich hinterher noch unterscheiden, ob ein Effekt aus der Szene stammt oder aus geänderter Belichtung bzw. verschobenen Hindernissen?
- **Kalibrierszenario:** woraus sollte später die intrinsische Kalibrierung entstehen, wenn nie ein bekanntes Muster durchs Bild wandert?
- **Dokumentation:** wüsste in zwei Wochen noch jemand, wer wann mit welchem Roboter welches Szenario aufgenommen hat und welche Topics enthalten sind?

> **Nicht vergessen:** Der Roboter wird für die Aufnahmen **manuell** per Teleoperation gefahren. Dafür brauchen Sie eine funktionierende Kette von der Tastatur bis `/cmd_vel`: entweder die eigene Teleop-Kette aus Übung 2 (wenn die noch nicht läuft, ist jetzt der Moment dafür) oder das fertige Paket `turtlebot3_teleop` mit `ros2 run turtlebot3_teleop teleop_keyboard`. Das ist deutlich weniger Aufwand, muss allerdings vorher noch installiert werden.

### Kurzvorstellung: 2 Minuten pro Gruppe

Jede Gruppe stellt ihre Ideen vor:

- **Keine Präsentation nötig.** Ein Zuruf, ein Flipchart oder ein Blick auf die eigene Stichpunktliste reicht aus.
- **Das ist kein „so machen wir das“-Vortrag**, sondern ein **„wir denken, das könnte gut funktionieren“**. Sie holen sich Feedback, bevor Sie Zeit in die Aufnahme stecken.
- **Zwei Minuten.** Halten Sie sich kurz und bringen Sie Ihre Punkte prägnant herüber.

Struktur, die funktioniert:

1. **Szenarien:** Was wollen Sie aufnehmen und wofür? (ein Satz pro Szenario)
2. **Topics:** Was muss dafür mit in den Bag?
3. **Offene Fragen:** Wo sind Sie unsicher und möchten eine Einschätzung?

> **Für die Zuhörenden:** Notieren Sie sich pro Gruppe (a) eine Idee, die Sie übernehmen wollen, und (b) eine Lücke, die Sie sehen. Beides brauchen wir gleich in der Diskussion.

### Gemeinsame Diskussion

Nach allen Kurzvorstellungen sammeln wir das Wichtigste für den ganzen Kurs. Ziel ist **ein gemeinsames Verständnis** und nicht ein einziger Bag für alle.

     {{1}}
************************************

**Leitfragen:**

1. **Mehrfachnutzen:** Welche Aufnahme deckt mehrere Blöcke ab? Eine Fahrt mit Kamera, LiDAR, IMU und Odometrie kann Block 2, 4 und 5 bedienen.
2. **Nicht nachholbar:** Welche Aufnahmen lassen sich später kaum wiederholen? Das betrifft etwa Szenarien, für die der Roboter belegt ist, der Aufbau aufwändig war oder die Bedingungen (Licht, Personen) nicht reproduzierbar sind. Solche Aufnahmen kommen zuerst.
3. **Vergleichbarkeit:** Welche Topics sollte **jedes** Team aufnehmen, damit Ergebnisse zwischen den Gruppen vergleichbar sind?
4. **Grenzen der Aufnahme:** Welche Fragen lassen sich mit Bags grundsätzlich nicht beantworten? <!--- (z. B. Regelverhalten auf Störgrößen, Latenz der gesamten Kette) --->
5. **Ablage:** Wo liegen die Bags, wer hat Zugriff, wie werden sie benannt? Aufgrund der Größe von Bags empfehlen wir einen **manuellen Transfer** (z.B. USB-Stick) statt Git.

************************************

     {{2}}
**Ergebnis der Diskussion:** eine gemeinsame Liste an Topics und Szenarien, welche für alle relevant aber nicht verpflichtend sind.

> **Wichtig:** Wenn Sie sich für etwas entscheiden, halten Sie es schriftlich fest. Sonst diskutieren Sie in zwei Wochen dieselbe Frage noch einmal.

### Reihenfolge und Aufnahme

--{{0}}--
Jetzt organisieren Sie sich untereinander: Wer fängt an, wer kommt danach? Denken Sie daran, dass Sie zwei Testkurse zur Verfügung haben!

     {{1}}
************************************

**Reihenfolge festlegen**

Damit die Reihenfolge nicht ausdiskutiert werden muss:

- Wer noch **nicht fahrbereit** ist, kommt später und nutzt die Wartezeit sinnvoll für das Setup.
- Planen Sie lieber **eine Aufnahme zu viel** als eine zu wenig: Ein zweiter Durchlauf am selben Tag ist billiger als ein neuer Termin.

************************************

     {{2}}
************************************

**Während der Aufnahme**

- **Eine Gruppe fährt, alle anderen halten die Strecke frei.** Hindernisse und Schilder werden **vor** dem Start aufgebaut, nicht während der Fahrt.
- **Vor dem Start prüfen:** `ros2 topic list`, `ros2 topic hz /scan` und `ros2 topic hz /camera/image_compressed`, dazu den Ladezustand des Akkus. Prüfen Sie auch, ob die Discovery der Bag Topics abgeschlossen ist.
- **Nach dem Start prüfen:** `ros2 bag info` muss Dauer, Nachrichtenzahl und Topics bestätigen. Fehlt ein Topic, muss der Bag vielleicht verworfen und neu gestartet werden.
- **Bags benennen:** `ros_ws/bags/<datum>_<teamname>_<domain id>_<szenario>/` und direkt nach der Aufnahme per Datentransfer sichern.
- **Keine Aufnahme ohne Notiz:** Szenario, Roboter und Besonderheiten in `ros_ws/bags/README.md` eintragen.

************************************

     {{3}}
************************************

**Was machen die anderen?**

- Offene Punkte aus **Übung 2** erledigen: Command-Velocity-Node, Parameterdatei, Launchdatei. Ohne funktionierende Teleoperation fährt kein Roboter.
- Den eigenen Roboter für den nächsten Slot vorbereiten: Netzwerk, `ROS_DOMAIN_ID`, Topic-Check, Akku.
- Die eigenen Nodes gegen einen **fremden Bag** testen und erste Auswertungen vorbereiten.

************************************

## Checkliste und Ausblick

     {{1}}
************************************

**Organisation**

- [ ] Gruppe steht, Repository-Verantwortliche:r ist benannt.
- [ ] Workspace mit Paketordner in `src/` vorhanden, `colcon build` läuft fehlerfrei.
- [ ] Remote-Repository angelegt und gepusht, alle Teammitglieder haben geklont.
- [ ] `README.md` enthält Team, Mitglieder sowie Build- und Startbefehle.

************************************

     {{2}}
************************************

**Bag-Aufnahme**

- [ ] Planung im Team abgeschlossen (Szenarien, Topics, Dauer).
- [ ] Kurzvorstellung gehalten, Feedback notiert.
- [ ] Ergebnis der Kursdiskussion zur Kenntnis genommen (gemeinsame Topics und Szenarien).
- [ ] Zeitslot bekannt, Aufnahme durchgeführt und mit `ros2 bag info` geprüft.
- [ ] Bag per Datentransfer gesichert und in `ros_ws/bags/README.md` dokumentiert.

************************************

     {{3}}
************************************

**Bis zum nächsten Praktikum**

- Fehlende Aufnahmen nachholen. Sprechen Sie das mit der Betreuung ab und setzen Sie nicht einfach auf den Roboter.
- Sicherstellen, dass die Teleoperation läuft. Sie ist das Fahrwerk für alle weiteren Aufnahmen.
- Prüfen, ob Sie Ihren Bag **ohne Roboter** abspielen und in `rviz2` visualisieren können: Image, LaserScan und eine Punktwolke sollten sichtbar sein.

************************************

<!-- kurslandkarte:ausblick-start -->

--------------------------------------------------------------------------------

**🗺️ Nächster Halt:** VL 04 · Sensorik (Mi 2.12. · 11:30–13:00) – Sie wissen, was die Sensoren des TurtleBots messen und wo ihre Grenzen liegen. [Auf der Kurslandkarte ansehen](https://tubaf-ifi-liascript.github.io/VL_SoftwareprojektRobotik/kurslandkarte/#termin=01)

<!-- kurslandkarte:ausblick-ende -->
