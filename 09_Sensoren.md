<!--

author:   Sebastian Zug & Georg Jäger
email:    sebastian.zug@informatik.tu-freiberg.de & Georg.Jaeger@informatik.tu-freiberg.de
version:  1.0.0
language: de
comment:  In dieser Vorlesungen werden die Schichten einer Roboterarchitektur adressiert.
narrator: Deutsch Female

import: https://raw.githubusercontent.com/LiaTemplates/Rextester/master/README.md
import: https://raw.githubusercontent.com/liascript-templates/plantUML/master/README.md
-->

# Vorlesung IX - Sensoren

Eine interaktive Version des Kurses finden Sie unter [Link](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/SoftwareprojektRobotik/master/09_ROS2_Sensoren.md)

**Zielstellung der heutigen Veranstaltung**

+ Typen von Sensoren in mobilen Robotern
+ Integration unter ROS
+ Vorverarbeitung

--------------------------------------------------------------------------------

# Sensoren

__Aufgabe:__ 	Gewinnung von Information über internen („Propriozeption“) 	bzw. externen Zustand  („Exterozeption“)  = „Wahrnehmung“ von 	Eigenzustand und Umwelt;

__Zielstellung:__ Möglichkeit zur Reaktion auf innere und äußere Einflüsse

## Sensorik des Menschen

| Klassifikation | Umsetzung                                  |
| -------------- | ------------------------------------------ |
| Modalitäten    | + Sehen, Hören, Riechen, Schmecken, Fühlen |
|                | + Temperatur, Gleichgewicht                |
|                | + Hunger, Durst                            |
| Qualitäten     | + rot, grün, blau                          |
|                | + süß, sauer, salzig, bitter               |
|                | + ungefähr 7 Grundgerüche                  |
| Intensität     | Amplitude                                  |

Und wie funktioniert das? Sogenannten Rezeptoren ...

... sind spezialisierte Zellen, die von bestimmten inneren oder äußeren Reizen angeregt werden und sie dann in Form von elektrischen Impulsen oder chemischen Reaktionen weiterleiten.

.. ein ausreichend starker Reiz bewirkt eine Veränderung des Membranpotentials (Generatorpotential)

... lösen ab einer gewissen Intensität des Reizes ein Schmerzempfinden aus.

| Rezeptoren        | Reiz                    |
| ----------------- | ----------------------- |
| Mechanorezeptoren | mechanische Deformation |
| Thermorezeptoren  | Temperaturänderung      |
| Photorezeptoren   | Licht                   |
| Chemorezeptoren   | Geschmäcker und Gerüche |

## Technische Sensoren

+ Ein Sensor (lateinisch „fühlen“ ), (Messgrößen-)aufnehmer oder (Mess-)Fühler ist ein technisches Bauteil, das bestimmte physikalische oder chemische Eigenschaften  … qualitativ oder als Messgröße quantitativ erfassen kann. Diese Größen werden mittels physikalischer oder chemischer Effekte erfasst und in ein weiterverarbeitbares elektrisches Signal umgeformt. [Wikipedia]

+ DIN 1319 1-4 … vermeidet den Begriff und spricht stattdessen in Abschnitt 2 vom „Messaufnehmer“ als dem Beginn der Messkette

+ Gesamtheit der Verfahren und Geräte zur experimentellen  Bestimmung und Verarbeitung zahlenmäßig erfassbarer Größen (DIN 1319)

> Sensoren transformieren physikalische, chemische oder biologische Messgrößen in elektrische Signale und stellen damit das unmittelbare Interface eines Messsystems zur Umgebung dar.


![RoboterSystem](./img/09_Sensoren/Fliehkrafregler.png)<!-- width="60%" -->
*Fliehkraftregler als Beispiel für die nicht elektrische Ausgabe von Messungen (Drehzahl).* [Wikipedia Commons, Nutzer: Kino]


![RoboterSystem](./img/09_Sensoren/SensorIntegrationsLevel.png)<!-- width="60%" -->
*Integrationsstufen von Sensoren* ["Architektur für verteilte, fehlertolerante Sensor-Aktor-Systeme", Sebastian Zug]

## Entfernungsmessung



## Aufgabe der Woche

+ Schreiben Sie einen Knoten, der Bilddaten erfasst und Bereiche mit meinen bestimmten Farbwert extrahiert. Nutzen Sie dafür OpenCV.
