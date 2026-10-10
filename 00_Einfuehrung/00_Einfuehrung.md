<!--

author:   Sebastian Zug & Georg Jäger & Copilot
email:    sebastian.zug@informatik.tu-freiberg.de & Georg.Jaeger@informatik.tu-freiberg.de
version:  0.2.0
language: de
narrator: Deutsch Female

import:   https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/config.md
  
-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/00_Einfuehrung/00_Einfuehrung.md)

# Einführung

<!-- data-type="none" -->
| Parameter            | Kursinformationen                                                                                             |
| -------------------- | ------------------------------------------------------------------------------------------------------------- |
| **Veranstaltung:**   | @config.lecture                                                                                               |
| **Semester**         | @config.semester                                                                                              |
| **Hochschule:**      | `Technische Universität Freiberg`                                                                             |
| **Inhalte:**         | `Einführung in die mobile Robotik`                                                                            |
| **Link auf GitHub:** | https://github.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/blob/master/00_Einfuehrung/00_Einfuehrung.md |
| **Autoren**          | @author                                                                                                       |

![](https://media.giphy.com/media/1flAwtHCYosL6LWnHr/giphy-downsized.gif)

--------------------------------------------------------------------------------

**Zielstellung der heutigen Veranstaltung**

+ Ablauf der Veranstaltung (Vorlesung, Übung, Praktikum) und die Projektaufgabe kennen
+ Grundlegende Begriffe und Definitionen der Robotik verstehen
+ Klassifikation von Robotersystemen anhand verschiedener Kriterien
+ Herausforderungen bei der Entwicklung autonomer mobiler Roboter kennenlernen
+ Systemarchitektur eines Roboters (Sense-Think-Act-Paradigma) verstehen
+ Motivation für den Einsatz von ROS 2 nachvollziehen

--------------------------------------------------------------------------------

## Unser Ziel: Autonomes Fahren im Miniaturformat

    --{{0}}--
Bevor wir über Definitionen sprechen, schauen wir auf das Ziel dieser Veranstaltung. Am Ende des Sommersemesters soll Ihr TurtleBot selbstständig durch eine kleine Verkehrswelt fahren: Er folgt der Straße, hält an der roten Ampel, beachtet Verkehrszeichen, lässt Fußgänger passieren und parkt am Ende ein. Alles, was Sie in Vorlesung und Übung lernen, ist ein Baustein für dieses Gesamtsystem.

!?[TurtleBot in der Verkehrswelt](https://github.com/user-attachments/assets/9ccb6e25-ef94-4707-b858-812347618593 "TurtleBot 3 im Praktikum: Fahrspur, Parkbuchten und Verkehrszeichen")

**Die Projektaufgabe**

Ein TurtleBot 3 Burger bewegt sich autonom in einer Miniatur-Verkehrswelt auf schwarzem Untergrund und muss dabei

+ **Straßen und Fahrspuren** erkennen und ihnen folgen,
+ **Ampelanlagen** und **Verkehrszeichen** erkennen und befolgen,
+ **Fußgängern** und Hindernissen ausweichen bzw. vor ihnen anhalten,
+ **einparken** und weitere Fahrmanöver ausführen.

    --{{1}}--
Diese Aufgabe lösen Sie nicht an einem Stück, sondern Block für Block. Jeder Block der Veranstaltung liefert eine Teilfähigkeit, die Sie im Praktikum direkt auf dem Roboter umsetzen.

                   {{1}}
*******************************************************************************

<!-- data-type="none" -->
| Block | Thema                              | Beitrag zur Projektaufgabe                          | Semester |
| ----- | ---------------------------------- | --------------------------------------------------- | -------- |
| 1     | ROS Einführung und Grundlagen      | Roboter ansteuern, Daten aufzeichnen und auswerten  | WiSe     |
| 2     | Sensorgrundlagen                   | Kamerabild korrigieren, Straße/Fahrspur erkennen    | WiSe     |
| 3     | Objekterkennung                    | Ampeln, Schilder, Fußgänger und Hindernisse erkennen | WiSe     |
| 4     | Sensorfusion                       | Position und Umgebung robust schätzen               | SoSe     |
| 5     | Regelung und Navigation            | Bahnfolge, Karte, Einparken                         | SoSe     |
| 6     | Praktikum + X                      | Integration zum Gesamtsystem                        | SoSe     |

*******************************************************************************

## Wie läuft die Veranstaltung ab?

    --{{0}}--
Anders als in den vergangenen Jahren trennen wir Vorlesung und Projekt nicht mehr auf zwei Semester. Vorlesung, Übung und Praktikum greifen von Anfang an ineinander. Jeder Termin dauert 90 Minuten. In manchen Wochen findet keine Vorlesung statt; diese Zeit gehört dann ganz der Arbeit am Roboter.

<!-- data-type="none" -->
| Format             | Wo?                           | Was passiert?                                                        |
| ------------------ | ----------------------------- | -------------------------------------------------------------------- |
| **Vorlesung (VL)** | Hörsaal                       | Konzepte und Methoden, Beispiele aus realen Robotersystemen          |
| **Übung (Ü)**      | eigener Rechner / Pool        | angeleitete Aufgaben, z. B. mit aufgezeichneten Sensordaten (Bags)   |
| **Praktikum (P)**  | Labor, TurtleBot              | Gruppenarbeit (2–3 Personen pro Roboter) an der Projektaufgabe      |

    --{{1}}--
Für die Arbeit benötigen Sie ein Linux-System. Wir arbeiten mit Ubuntu 24.04 und ROS 2 Jazzy. Wie Sie das auf Ihrem Rechner einrichten, klären wir in den ersten Übungen.

     {{1}}
*******************************************************************************

**Werkzeuge**

+ Ubuntu 24.04 (nativ, alternativ WSL2 unter Windows)
+ ROS 2 Jazzy
+ Python und C++
+ Git / GitHub für die Gruppenarbeit — ich möchte in dieser Veranstaltung eine intensive Nutzung der Versionsverwaltung sehen!

*******************************************************************************

**Wo stehen Sie?** Ihre Antworten helfen uns, die Übungen und die Gruppen passend zu planen.

Wie sicher sind Sie im Umgang mit dem Linux-Terminal?

[(sicher)] sehr sicher
[(grundlagen)] grundlegende Erfahrung
[(keine)] kaum / keine Erfahrung

Mit welchen Themen haben Sie schon gearbeitet?

[[python]] Python
[[cpp]] C++
[[git]] Git
[[ros]] ROS / ROS 2
[[opencv]] OpenCV / Bildverarbeitung
[[embedded]] Mikrocontroller / Embedded

## Was ist ein Roboter?

    --{{0}}--
Die Geschichte der Robotik ist faszinierend und reicht weit zurück. Bereits 1921, also vor über 100 Jahren, wurden in Filmen wie "The Mechanical Man" mechanische Wesen dargestellt, die autonom handeln können. Diese frühen Visionen haben die Entwicklung der modernen Robotik maßgeblich beeinflusst.

Das Wort wurde 1920 vom Schriftsteller Karel Čapek geprägt. Seine Bedeutung hat sich aber im Laufe der Zeit gewandelt und ist gleichzeitig vielschichtig.

![Mecanical Man](https://upload.wikimedia.org/wikipedia/commons/3/3c/L%27uomo_meccanico_1.png "Screenshot aus dem Film [Mechanical Man](https://en.wikipedia.org/wiki/The_Mechanical_Man) von 1921")

Es gibt keine einheitliche Definition des Begriffes Roboter. Die offiziellen Definitionen gehen weit auseinander. 

> **Definition nach VDI-Richtlinie 2860**: „Industrieroboter sind universell einsetzbare Bewegungsautomaten mit mehreren Achsen, deren Bewegungen hinsichtlich Bewegungsfolge und Wegen bzw. Winkeln frei (d. h. ohne mechanischen bzw. menschlichen Eingriff) programmierbar und gegebenenfalls sensorgeführt sind. Sie sind mit Greifern, Werkzeugen oder anderen Fertigungsmitteln ausrüstbar und können Handhabungs- und/oder Fertigungsaufgaben ausführen.“

> **Definition nach Robotic Industries Association (RIA)**: „A robot is a reprogrammable, multifunctional manipulator designed to move material, parts, tools or specialized devices through variable programmed motions for the performance of a variety of tasks“

> **Japan Robot Association (JRA)**
>
> - Manual Manipulator: Handhabungsgerät, das kein Programm hat, sondern direkt vom Bediener geführt wird,
> - Fixed Sequence Robot: Handhabungsgerät, das wiederholt nach einem konstanten Bewegungsmuster arbeitet. Das Ändern des Bewegungsmusters ist relativ aufwendig,
> - Variable Sequence Robot: Handhabungsgerät, wie vorher beschrieben, jedoch mit der Möglichkeit, den Bewegungsablauf schnell und problemlos zu ändern,
> - Playback Robot: Der Bewegungsablauf wird diesem Gerät einmal durch den Bediener vorgeführt und dabei im Programmspeicher gespeichert. Mit der im Speicher enthaltenen Information kann der Bewegungsablauf beliebig wiederholt werden,
> - Numerical Control Robot: Dieses Handhabungsgerät arbeitet ähnlich wie eine NC-gesteuerte Maschine. Die Information über den Bewegungsablauf wird dem Gerät über Taster, Schalter oder Datenträger zahlenmäßig eingegeben,
> - Intelligent Robot: Diese höchste Roboterklasse ist für Geräte gedacht, die über verschiedene Sensoren verfügen und damit in der Lage sind, den Programmablauf selbsttätig den Veränderungen des Werkstücks und der Umwelt anzupassen.



## Klassifikation von Robotersystemen

    --{{0}}--
Roboter lassen sich anhand verschiedener Kriterien klassifizieren. Diese Unterscheidungsmerkmale helfen uns, die Vielfalt der Robotersysteme zu verstehen und ihre spezifischen Anforderungen zu erkennen. In dieser Veranstaltung konzentrieren wir uns vor allem auf autonome, mobile Roboter mit komplexer Sensorik.

**Welche Robotersysteme kennen Sie aus Ihrer Praxis?**

<!-- data-type="none" -->
| **Kriterium**                | **Optionen**                                                  |
| ---------------------------- | ------------------------------------------------------------- |
| **Art der Steuerung**        | <span style="color:blue">autonom</span>, teleoperiert, hybrid |
| **Bewegungsfähigkeit**       | stationär, <span style="color:blue">mobil</span>              |
| **Anwendungsbereich**        | Industrie, Verkehr, Medizin ...                               |
| **Erscheinung**              | Humanoid, Nicht-humanoid                                      |
| **Energieversorgung**        | Autark, Batterien, Kabelgebunden                              |
| **Interaktionsfähigkeit**    | Kooperativ, Isoliert                                          |
| **Komplexität der Umgebung** | Niedrig, Hoch                                                 |
| **Sensorik und Wahrnehmung** | Einfach, <span style="color:red">Komplex</span>               |
| **Größe**                    | Mikroroboter, Makroroboter                                    |
| ...                          | ...                                                           |

    --{{0}}--
In dieser Veranstaltung konzentrieren wir uns primär auf autonome, mobile Roboter, die in komplexen Umgebungen agieren müssen. Diese Systeme stellen besondere Anforderungen an Sensorik, Datenverarbeitung und Entscheidungsfindung.

**Selbsttest:** Ordnen Sie die Systeme zu. Welche Eigenschaften treffen zu?

[ [autonom] [mobil] [komplexe Sensorik] ]
[    [X]      [X]          [ ]       ] Mähroboter, der sich an einem Begrenzungsdraht orientiert
[    [ ]      [ ]          [ ]       ] Schweißroboter an einer Fertigungsstraße mit festem Programm
[    [ ]      [X]          [X]       ] ferngesteuerter Inspektionsroboter mit Kamera und LiDAR
[    [X]      [X]          [X]       ] unser TurtleBot in der Verkehrswelt
***
Autonomie und Mobilität sind voneinander unabhängig: Der ferngesteuerte Inspektionsroboter ist mobil und reich an Sensoren, trifft aber keine eigenen Entscheidungen. Der Mähroboter entscheidet selbst, kommt aber mit sehr einfacher Sensorik aus. Unser TurtleBot muss alle drei Eigenschaften vereinen.
***

## Relevanz und Bedeutung mobiler Roboter

    --{{0}}--
Die Bedeutung mobiler Roboter hat in den letzten Jahrzehnten massiv zugenommen. Dies lässt sich anhand der Häufigkeit von Begriffen in wissenschaftlichen Publikationen nachvollziehen. Die folgenden Grafiken zeigen die Entwicklung der Begriffe "Autonomous Vehicle" und "Autonomous Robot" sowie verwandter KI-Themen. 

??[IFrame](https://books.google.com/ngrams/graph?content=Autonomous+Vehicle%2CAutonomous+Robot&year_start=1900&year_end=2022&corpus=en&smoothing=3 "Ngram Analyse der Begriffe Autonomous Vehicle und Autonomous Robot")

??[IFrame](https://books.google.com/ngrams/graph?content=artificial+intelligence%2Crobot%2CMachine+Learning%2C+deep+learning&year_start=1900&year_end=2022&corpus=en&smoothing=3 "Ngram Analyse der Begriffe Artificial Intelligence, Robot, Machine Learning und Deep Learning")

    --{{0}}--
Besonders auffällig ist der exponentielle Anstieg ab den 2010er Jahren, der mit Fortschritten im Machine Learning und Deep Learning zusammenfällt. Diese Technologien sind heute unverzichtbare Werkzeuge für autonome Robotersysteme.

## Herausforderungen bei der Entwicklung mobiler Roboter

    --{{0}}--
Die Entwicklung autonomer mobiler Roboter ist eine hochkomplexe Aufgabe, die Herausforderungen auf mehreren Ebenen mit sich bringt. Schauen wir uns diese systematisch an. 


**Welche technologischen Herausforderungen müssen wir bei mobilen Robotersystemen meistern?**

    --{{0}}--
Auf der technologischen Ebene stehen wir vor fundamentalen Problemen: Wie kann ein Roboter seine Position präzise bestimmen? Wie erstellt er eine Karte seiner Umgebung? Wie erkennt und umfährt er Hindernisse? All diese Fragen werden wir im Verlauf dieser Veranstaltung adressieren.

                  {{0-4}}
*******************************************************************************

**Technologische Herausforderungen**

+ Robuste, hinreichend präzise **Positionierung**
+ **Umgebungskartierung** (SLAM - Simultaneous Localization and Mapping)
+ **Hindernisidentifikation** und -umgehung
+ **Echtzeit-Umsetzung** von Teilverhalten
+ **Energieeffizienz** für längere Betriebszeiten
+ Umgang mit **veränderlichen Kommunikationsbedingungen**

*******************************************************************************

    --{{1}}--
Neben der technischen Realisierbarkeit müssen Robotersysteme auch wirtschaftlich sinnvoll sein. Die Entwicklungskosten müssen sich durch den Nutzen rechtfertigen lassen, und die Systeme müssen marktreif sein.

                  {{1-4}}
*******************************************************************************

**Wirtschaftliche Herausforderungen**


+ **Wirtschaftlichkeit**: Kosten-Nutzen-Verhältnis
+ **Marktreife**: Zuverlässigkeit für den produktiven Einsatz
+ **Skalierbarkeit**: Vom Prototyp zur Serie

*******************************************************************************


    --{{2}}--
Schließlich dürfen wir die gesellschaftlichen Aspekte nicht vergessen. Autonome Systeme müssen sicher sein, rechtlichen Vorgaben entsprechen und von der Gesellschaft akzeptiert werden. Ethische Fragen wie der Umgang mit Datenschutz oder potenzielle Arbeitsplatzverdrängung sind hochrelevant.

                  {{2-4}}
*******************************************************************************

**Soziale und rechtliche Herausforderungen**

+ **Sicherheitsanforderungen**: Schutz von Menschen und Infrastruktur
+ **Regulatorische Rahmenbedingungen**: Gesetze und Normen
+ **Ethik und Datenschutz**: Verantwortungsvoller Umgang mit Daten
+ **Gesellschaftliche Akzeptanz**: Arbeitsplatzverdrängung, Vertrauen

*******************************************************************************

    --{{3}}--
Betrachten wir zwei konkrete Beispiele aus der Praxis: einen Lieferroboter für städtische Umgebungen und einen aquatischen Roboter für Gewässerüberwachung. Welche der genannten Herausforderungen sind jeweils besonders relevant?

                  {{3-4}}
*******************************************************************************

> **Diskussion:** Welche dieser Herausforderungen sehen Sie als besonders relevant für folgende Szenarien?
> 
> 1. **Lieferroboter** in einer Fußgängerzone
> 2. **Aquatischer Roboter** zur Gewässerüberwachung 

![Husky](images/Roboter_Engstelle2.png "Autonomer Roboter des Ready for Robots Projektes")
![Schwimmroboter](images/Schwimmroboter.png "TUBAF Schwimmroboter mit Windmessungsaufsatz")

*******************************************************************************

## Systemarchitektur: Das Sense-Think-Act-Paradigma

    --{{0}}--
Wie ist nun ein autonomer Roboter aufgebaut? Die klassische Architektur folgt dem Sense-Think-Act-Paradigma, das sich in drei hierarchische Ebenen gliedert. Auf der untersten Ebene finden wir die reaktive Überwachung, die in Echtzeit auf Sensordaten reagiert. Darüber liegt die Ausführungsebene, die taktische Entscheidungen trifft. An der Spitze steht die Handlungsplanung, die strategische Ziele verfolgt. 
Diese Darstellung zeigt die grundlegende Dreischichten-Architektur. Sensoren erfassen Daten aus der Umgebung, die reaktive Überwachung verarbeitet diese in Echtzeit und generiert direkte Steuerbefehle an die Aktoren. Diese Ebene arbeitet mit sehr kurzen Reaktionszeiten.


                   {{0-1}}
*************************************************

```ascii
                    Statusmeldungen 
     Nutzereingaben  ^                                       
                 |   |
Befehle          v   |
            +-----------------------+
            | Handlungsplanung      |  "$Strategie   $"
            +-----------------------+
                 |   ^      | | |       Folge von Aktionen     
                 v   |      v v v
            +-----------------------+
            | Ausführung            |  "$Taktik$    "           
            +-----------------------+
                     ^      | | |       Geplante Trajektorie,
Status               |      v v v       Verhalten
            +-----------------------+
            | Reaktive Überwachung  |  "$Ausführung$        "
            +-----------------------+
Sensordaten-    ^ ^ ^       | | |       Steuerbefehle an den 
erfassung       | | |       v v v       Aktuator 
            +----------+ +----------+
            | Sensoren | | Aktoren  |                               
            +----------+ +----------+
                  ^           |
                  |           v      
            .-----------------------.
            | Umgebung              |
            .-----------------------.                                                                                .
```

*************************************************

    --{{1}}--
In der erweiterten Darstellung sehen wir, welche konkreten Aufgaben auf den einzelnen Ebenen anfallen. Die reaktive Überwachung kümmert sich um Regelung und Energiemanagement. Die Ausführungsebene verwaltet das Weltmodell und erkennt Kontexte. Die Handlungsplanung entwickelt Verhaltensmodelle und kommuniziert mit dem Nutzer.
Auf der rechten Seite sehen Sie die konkreten Implementierungsaspekte: Hardwaretreiber, Regelungsalgorithmen, Weltmodelle und Verhaltensplanung. Diese Komplexität macht deutlich: Ein vollständiges Robotersystem zu entwickeln ist eine enorme Aufgabe!

                   {{1-3}}
*************************************************

```ascii
                    Statusmeldungen 
     Nutzereingaben  ^                                                           
                 |   |                                                   Kommunikation, Nutzerschnittstellen
Befehle          v   |                                                                     
            +-----------------------+                                                                 
            | Handlungsplanung      |  "$Strategie   $"                  Verhaltensmodell                        
            +-----------------------+                                                                 
                 |   ^      | | |       Folge von Aktionen                                                      
                 v   |      v v v                                                                 
            +-----------------------+                                                                 
            | Ausführung            |  "$Taktik$    "                    Weltmodell, Handungsmuster Kontexterkennung
            +-----------------------+                                                                 
                     ^      | | |       Geplante Trajektorie,                                                        
Status               |      v v v       Verhalten                                                                 
            +-----------------------+                                                                 
            | Reaktive Überwachung  |  "$Ausführung$        "            Regelung, Energiemanagement                  
            +-----------------------+                                    Notaus-Erkennung                             
Sensordaten-    ^ ^ ^       | | |       Steuerbefehle an den                                             
erfassung       | | |       v v v       Aktuator                                                                  
            +----------+ +----------+                                                                 
            | Sensoren | | Aktoren  |                                    Hardwaretreiber                          
            +----------+ +----------+                                                                 
                  |           |                                                                 
                  v           v                                                                       
            .-----------------------.                                                                 
            | Umgebung              |                                                                 
            .-----------------------.                                                                                .
```

![RoboterSystem](images/RoboterNuernberg.jpg "Roboter des RoboCupTeams aus Nürnberg - TDP des Teams AutonOhm, 2019")

    --{{2}}--
Und genau hier stellt sich die entscheidende Frage: Wer soll das alles implementieren? Muss jedes Forschungsteam, jedes Unternehmen, jeder Entwickler wirklich das Rad neu erfinden?

      {{2}}
> **Zentrale Frage:** Wer soll das denn alles implementieren?

*************************************************

### Sense-Think-Act am TurtleBot

    --{{0}}--
Übertragen wir das Schema auf unseren eigenen Roboter. Der TurtleBot 3 Burger ist klein, aber er bringt alle drei Ebenen mit: Sensoren zur Wahrnehmung, einen Rechner für die Entscheidungen und Motoren für die Bewegung.

<!-- data-type="none" -->
| Komponente           | Spezifikation                              |
| -------------------- | ------------------------------------------ |
| LiDAR                | RPLIDAR C1 (360°, 12 m Reichweite, 10 Hz)  |
| Kamera               | Raspberry Pi Camera v2                     |
| Antrieb              | Differentialantrieb (DYNAMIXEL)            |
| Max. Geschwindigkeit | 0,22 m/s                                   |
| Rechner              | Raspberry Pi 3 Model B+                    |

**Selbsttest:** Auf welcher Ebene ist die jeweilige Aufgabe in unserer Verkehrswelt angesiedelt?

[ [Sense] [Think] [Act] ]
[  ( )     (X)     ( )  ] Entscheiden, ob bei Gelb noch gefahren wird
[  (X)     ( )     ( )  ] Kamerabild der Fahrbahn erfassen
[  ( )     ( )     (X)  ] Radgeschwindigkeiten über die Motoren einstellen
[  (X)     ( )     ( )  ] Abstand zu einem Fußgänger mit dem LiDAR messen
[  ( )     (X)     ( )  ] Einen Pfad in die Parklücke planen

## Warum ROS? Das "Rad nicht neu erfinden"

    --{{0}}--
Genau an diesem Punkt kommt das Robot Operating System ins Spiel. ROS ist nicht wirklich ein Betriebssystem im klassischen Sinne, sondern ein Framework, das Standardlösungen für wiederkehrende Robotik-Probleme bereitstellt. Statt alles von Grund auf neu zu entwickeln, können Entwickler auf eine riesige Bibliothek von Werkzeugen und Algorithmen zurückgreifen.
Dieser Comic von Willow Garage, dem ursprünglichen Entwickler von ROS, bringt das Problem auf den Punkt: Ohne ein gemeinsames Framework entwickelt jeder sein eigenes System, seine eigenen Treiber, seine eigenen Algorithmen. Das ist ineffizient, fehleranfällig und behindert den wissenschaftlichen Fortschritt.

![RoboterSystem](images/willow_p1_02s.640w.jpg "Willow Garage, http://www.willowgarage.com/blog/2010/04/27/reinventing-wheel, 2010")<!-- width="50%" -->


    --{{1}}--
ROS 2, die aktuelle Version, die wir in dieser Veranstaltung nutzen werden, bietet eine moderne, produktionsreife Plattform mit Echtzeit-Fähigkeiten, verbesserter Sicherheit und Unterstützung für verschiedene Betriebssysteme.

                        {{1-2}}
******************************************************

**ROS 2 bietet:**

+ **Standardisierte Kommunikation** zwischen Komponenten (Topics, Services, Actions)
+ **Wiederverwendbare Pakete** für Sensoren, Navigation, Bildverarbeitung, etc.
+ **Werkzeuge** zur Visualisierung, Datenaufzeichnung und Analyse
+ **Community-Support** mit tausenden von Entwicklern weltweit
+ **Echtzeit-Fähigkeiten** und **Sicherheit** (im Gegensatz zu ROS 1)

******************************************************

## Ausblick: Die nächsten Schritte

    --{{0}}--
Bevor wir mit ROS 2 arbeiten können, brauchen wir ein solides Fundament: den sicheren Umgang mit dem Linux-Terminal. Genau darum geht es in der ersten Übung. Danach steigen wir in der Vorlesung in die Konzepte von ROS 2 ein und richten in der folgenden Übung ROS 2 auf Ihren Rechnern ein.

**Was kommt als Nächstes?**

<!-- data-type="none" -->
| Termin                                | Inhalt                                                                  |
| ------------------------------------- | ----------------------------------------------------------------------- |
| **Ü** OS + Linux Shell                | Terminal, Dateisystem, Rechte, Prozesse, Umgebungsvariablen             |
| **VL** Grundlagen ROS 2               | Architektur, Knoten, Topics, Nachrichten                                |
| **VL** ROS 2 Pakete                   | Pakete, Build-System, Launch-Files                                      |
| **Ü** Installation / Grundlagen ROS 2 | ROS 2 Jazzy einrichten, erste Schritte mit der Kommandozeile            |
| **VL** Koordinaten + QoS              | Transformationen, tf2, Quality of Service                               |
| **P** Grundlagen ROS 2 + Bags         | erste Fahrt mit dem TurtleBot, Daten aufzeichnen und auswerten          |

    --{{1}}--
Wenn Sie noch kein Linux-System haben, nutzen Sie die Zeit bis zur Übung. Unter Windows ist das Linux-Subsystem WSL2 eine gute Alternative zu einer nativen Installation.

      {{1}}
> **Bis zur Übung:** Stellen Sie sicher, dass Sie Zugriff auf ein Ubuntu-24.04-System haben — nativ, als WSL2 unter Windows oder im Rechnerpool.

## Zusammenfassung

    --{{0}}--
Fassen wir die wichtigsten Punkte dieser Einführungsvorlesung zusammen:

**Was haben wir gelernt?**

+ Ziel der Veranstaltung ist ein TurtleBot, der **autonom durch eine Miniatur-Verkehrswelt** fährt — aufgebaut Block für Block in VL, Ü und P
+ Roboter sind **computergesteuerte Maschinen**, die **komplexe Aktionen automatisch** ausführen
+ Robotersysteme lassen sich anhand **vieler Kriterien klassifizieren** (Autonomie, Mobilität, Sensorik, etc.)
+ Die Entwicklung mobiler Roboter bringt **technologische, wirtschaftliche und soziale Herausforderungen** mit sich
+ Die **Systemarchitektur** folgt dem **Sense-Think-Act-Paradigma** mit drei Ebenen
+ **ROS 2** ermöglicht es uns, **auf vorhandenen Lösungen aufzubauen** statt alles neu zu entwickeln

    --{{1}}--
Damit kennen Sie das Ziel und den Weg dorthin. In der nächsten Übung legen wir mit dem Linux-Terminal das Fundament. Ich freue mich auf die gemeinsame Reise durch die faszinierende Welt der mobilen Robotik!

      {{1}}
> **Zum Nachdenken:** Welche Fähigkeiten braucht ein menschlicher Fahrer, um durch unsere Verkehrswelt zu kommen — und welcher Sensor des TurtleBots könnte jeweils die Augen und Ohren ersetzen?

## Weiterführende Ressourcen

    --{{0}}--
Für Interessierte hier noch einige Ressourcen zur Vertiefung:

**Literatur und Links:**

+ [ROS 2 Documentation](https://docs.ros.org/) - Offizielle ROS 2 Dokumentation
+ [The Robotics Back-End](https://roboticsbackend.com/) - Tutorials und Praxisbeispiele
+ [RoboCup](https://robocup.de/) - RoboCup-Wettbewerbe und -Teams
+ [TurtleBot 3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) - Dokumentation des Herstellers ROBOTIS

## Forschung und Praxis

    --{{0}}--
Abschließend noch ein Hinweis: Die Vorlesung vermittelt Grundlagen, aber die eigentliche Forschungs- und Entwicklungsarbeit beginnt erst danach. Im verlinkten Dokument finden Sie Beispiele aktueller Forschungsprojekte.

https://github.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/blob/master/00_Einfuehrung/images/ROSE2024_Chemnitz.pdf