<!--

author:   Sebastian Zug & Georg Jäger
email:    sebastian.zug@informatik.tu-freiberg.de & Georg.Jaeger@informatik.tu-freiberg.de
version:  0.0.1
language: de
narrator: Deutsch Female

import: https://raw.githubusercontent.com/LiaTemplates/Rextester/master/README.md

-->

# Vorlesung Softwareprojekt

Der vorliegende Kurs realisiert die Vorlesung "Softwareprojekt" an der TU Bergakademie
Freiberg auf der Basis von LiaScript. Eine interaktive Darstellung der Inhalte ist unter

[LiaScripLink](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/SoftwareprojektRobotik/master/README.md#1)

zu finden.

Die
Veranstaltung richtet sich an Informatiker und Mathematiker und adressiert die folgenden Punkte:

+ aufbauend auf dem vorangegangenen Kurs Softwareentwicklung werden die Konzepte von C++ erläutert
+ Einführung in das Robot Operating System (ROS)
+ Elemente von Robotersystemen und deren Implementierung wie

  + Grundlagen der Sensoren und deren Einbettung in ROS
  + Aktoren und deren Regelung
  + Karten, deren Erzeugung und Nutzung


+ Architekturkonzepte für Roboteranwendungen


| Datum      | Titel                  | GitHub-Link                                                                                  | LiaScript-Link                                                                                                                               |
|:-----------|:-----------------------|:---------------------------------------------------------------------------------------------|:---------------------------------------------------------------------------------------------------------------------------------------------|
| 14.10.2019 | Einführung             | [Link](https://github.com/SebastianZug/SoftwareprojektRobotik/blob/master/00_Einfuehrung.md) | [Link](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/SoftwareprojektRobotik/master/00_Einfuehrung.md#1) |
| 21.10.2019 | Klassen und Strukturen | [Link](https://github.com/SebastianZug/SoftwareprojektRobotik/blob/master/01_OOPinC++.md)    | [Link](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/SoftwareprojektRobotik/master/01_OOPinC++.md#1)    |


## Organisatorisches (WS 2019/20)

**Dozenten**

| Name          | Email                                   |
|:--------------|:----------------------------------------|
| Sebastian Zug | sebastian.zug@informatik.tu-freiberg.de |
| Georg Jäger   | georg.jaeger@informatik.tu-freiberg.de  |


**Zielstellung der Veranstaltung**

+ Anwendungsbereites Programmierverständnis unter C++
+ Basiskonzepte von ROS und den entsprechenden Entwicklungswerkzeugen
+ Verständnis für generelle Architekurkonzepte und Lösungsansätze in verschiedenen Bereichen der Robotik

... als Vorbereitung auf die praktischen Aufgaben im Sommersemester.

**Prüfungsleistungen**

Die Veranstaltung ist in zwei Teile gegliedert. Im Wintersemester werden Sie
zunächst mit den Grundlagen vertraut gemacht, um :

* als Informatiker am Ende eine mündliche Prüfung zu absolvieren und im Sommersemester eine praktische Aufgabe umzusetzen. Diese wird separat bewertet.
* als Mathematiker mit einer kleineren Aufgabe am Ende des Semester die Veranstaltung abzuschließen.

**Zeitaufwand und Engagement**

Der Zeitaufwand beträgt 180h und setzt sich zusammen aus 60h Präsenzzeit und
120h Selbststudium. Letzteres umfasst die Vor- und Nachbereitung der
Lehrveranstaltungen, die eigenständige Lösung von Übungsaufgaben sowie die
Prüfungsvorbereitung.

**Die eigene Beschäftigung mit der C++ Programmierung und den Konzepten von ROS ist für das erfolgreiche Bestehen der Veranstaltung unabdingbar!**

**Ablaufplan Übungen**

Übungstermine

| Nr. | Datum                                    |
|:----|:-----------------------------------------|
| 1.  | ~23./24. Oktober~ -> **6./7. November**  |
| 2.  | 13./14. November                         |
| 3.  | ~20./21. November~ -> **4./5. Dezember** |
| 4.  | 18./19. Dezember                         |
| 5.  | 15./16. Januar                           |
| 6.  | 29./30. Januar                           |

Tragt euch bitte in den Kurs [Softwaretechnologie-Projekt (Prototyp)](https://bildungsportal.sachsen.de/opal/auth/RepositoryEntry/18593513489/CourseNode/98504809493283) im e-learning System OPAL ein.

**Jedes Aufgabenblatt beinhaltet Vorbereitungsaufgaben die eigenständig bearbeitet und in der Übung lediglich verglichen werden!**

## Literaturempfehlungen

**Vorlesungen**

+ Vorlesung "Objektorientierte Programmierung mit C++ " der Universität Ulm (Dr. Andreas F. Borchert) [Link](https://www.uni-ulm.de/mawi/mawi-numerik/lehrenumerik/vergangene-semester/sommersemester-2018/vorlesung-objektorientierte-programmierung-mit-c/)
+ Vorlesung "Programmieren in C++" der Universität Freiburg (Frau Prof. Dr. Hannah Bast) [Link](https://ad-wiki.informatik.uni-freiburg.de/teaching/ProgrammierenCplusplusSS2018)

**Videotutorials**

+ Youtube Kanal von "The Cherno", [Link](https://www.youtube.com/playlist?list=PLlrATfBNZ98dudnM48yfGUldqGD0S4FFb)
+ Wikibooks "C++-Programmierung", [Link](https://de.wikibooks.org/wiki/C%2B%2B-Programmierung/_Inhaltsverzeichnis)

**Talks**

+ Stroustrup, Bjarne - "The Essence of C++" , [Link](https://www.youtube.com/watch?v=86xWVb4XIyE)
+ Stroustrup, Bjarne - "Learning and Teaching Modern C++", [Link](https://www.youtube.com/watch?v=fX2W3nNjJIo)

**Bücher**

+ Torsten T. Will, "C++ - Das umfassende Handbuch", Rheinwerk Computing 2019

## Ok, womit soll ich anfangen?

1. Legen Sie sich ein Repository an, mit dem Sie arbeiten!
2. Entscheiden Sie sich für Linux als Betriebssystem auf Ihrem Rechner :-), in diesem Fall sind alle Tools die Sie im Laufe des Semesters benötigen, sofort greifbar.
3. Starten Sie mit kleinen Beispielen, um Ihre Algorithmischen Fähigkeiten zu schulen und sich zum anderen mit der Semantik von C++ vertraut zu machen.

Wenn Sie Punkt 2. nicht uneingeschränkt folgen wollen, hätten Sie zumindest für die Einführung zu C++ unter Windows folgende Möglichkeiten:

| Toolchain       | Link                                                                                  | Bemerkung |
|:----------------|:--------------------------------------------------------------------------------------|:----------|
| Visual Studio   |                                                                                       |           |
| MinGW           | [MinGW Webseite](http://www.mingw.org/)                                               |           |
| Cygwin          | [Cygwin Webseite](https://cygwin.com/)                                                |           |
| Linux-Subsystem | [Microsoft Dokumentation](https://docs.microsoft.com/de-de/windows/wsl/install-win10) |           |

Unter cygwin muss zunächst der Compiler `gcc-g++` installiert werden. Für Rekonfigurationen und neues Pakete führen Sie einfach den Installer nochmals aus. Zudem sollten Sie in Ihrem Home einen symbolischen Link anlegen, der auf Ihren Arbeitsordner verweist.

```
$ ln -s /cygdrive/c/Users/Sebas/OneDrive/Desktop/Softwareprojekt/ mySoftware
```

![Atom IDE Screenshot](./img/00_Einfuehrung/CygwinConfiguration.png)<!-- width="100%" -->

![Atom IDE Screenshot](./img/00_Einfuehrung/ScreenshotCygwinConfiguration.png)<!-- width="100%" -->


Hinsichtlich Ihres Lieblingseditors haben Sie sicher schon eine Wahl getroffen :-)

## Ein Wort zu LiaScript

Für den ersten Punkt wird der Quellcode direkt in das LiaScript-Dokument eingebettet und kann damit im Browser bearbeitet werden. Dabei ist ein "Versionssystem light" verfügbar, das Änderungen lokal im Browser abspeichert. Basis für die Ausführungsumgebung ist die API von Rextester https://rextester.com/. Vielen Dank für diesen Service!

Ausführbarer C++ Code sieht wie folgt aus, der Titel kann weggelassen werden.

```cpp                     HelloWorld.cpp
#include <iostream>

int main()
{
    using namespace std;
    cout << "Hello, World!";
    return 0;
}
```
@Rextester.CPP

Der Markdown-Quellcodeblock wird dafür um den Makroaufruf `@Rextester.CPP` am
Ende ergänzt. Dabei können unterschiedliche Compiler (gcc, clang, Microsoft C++ Compiler) und Parametersets genutzt werden.

```
@Rextester.CPP
@Rextester.CPP_clang
@Rextester.CPP_vc
```

Im Weiteren ist es möglich "Kommandozeilenparameter" an die Rextester-Ausführungsumgebung
zu übergeben und diese dann auch im Browser zu variieren. In diesem Fall werden
die Parameter in einem separaten Codeblock angefügt, gefolgt von der Anweisung
`@Rextester.CPP(true,@input(1))`

```cpp                     HelloWorld.cpp
#include <iostream>

int main()
{
    using namespace std;
    int input{};
    int output{};

    cout << "Bitte geben Sie eine Zahl ein!" << endl;
    cin >> input;
    output = input * 2;
    cout << "Das Doppelte dieser Zahl ist " << output << "." << endl;
    return 0;
}
```
```
5
```
@Rextester.CPP(false,`@input(1)`)

Der boolsche Parameter kann genutzt werden um eine knappe Statistik der Kompilierung
und der Ausführungsparameter abzurufen.

Darüber hinaus können auch komplette Parameterset an den Compiler übergeben werden, um zum Beispiel die verschiedenen C++ Versionen zu evaluieren. Wenn Sie zum Beispiel in folgendem Beispiel, dass mit dem Makroaufruf

```
@Rextester.eval(@CPP, false, ,`-Wall -std=c++98 -O2 -o a.out source_file.cpp`)
```

kompiliert wird, versuchen `auto` und die uniforme Initialisierung zu nutzen (jeweils ab C++11), generiert der Compiler einen Fehler.

```cpp                     HelloWorld.cpp
#include <iostream>

int main()
{
    using namespace std;
    int output = 5;
    //auto output {5};
    cout << output;
    return 0;
}
```
@Rextester.eval(@CPP, false, ,`-Wall -std=c++98 -O2 -o a.out source_file.cpp`)

## Wie können Sie zum Gelingen der Veranstaltung beitragen?

* Stellen Sie Fragen, seinen Sie kommunikativ!
* Organisieren Sie sich in Arbeitsgruppen!
* Experimentieren Sie mit verschiedenen Entwicklungsumgebung um "Ihren Editor"
  zu finden
* Machen Sie Verbesserungsvorschläge für die Vorlesungsfolien!

![Atom IDE Screenshot](./img/00_Einfuehrung/LiaScriptAtomScreenShot.png)<!-- width="100%" -->

## Fragebogen zum Lernfortschritt

Die Arbeitsgruppe ist in verschiedenen Projekten rund um die Digitalisierung des Lernens an Hochschulen involviert, die durch das Bundesministerium für Bildung und Forschung getragen werden, beteiligt [Industrial-eLab](http://www.elab.ovgu.de/). Entsprechend wollen wir gemeinsam mit Partnern der Hochschule Anhalt verstehen, wie Sie in Ihrem Lernprozess vorgehen und wie wir Sie ggf. unterstützen können.

Beispiel aus dem Projekt:

![Atom IDE Screenshot](./img/00_Einfuehrung/LearningAnalyticsExample.png)<!-- width="100%" -->
*Gegenüberstellung des Programmierverhaltens eines erfahrenen und eines weniger erfahrenen Studenten*
[^1]: [Quelle](http://www.elab.ovgu.de/elab_media/_users/hawlitsc/Paper_FDIBA_eLab/FDIBA_eLab-p-90.pdf)

**Die Erkenntnisse aus dieser Untersuchung bleiben anonym und beeinflussen in keinster Weise Ihr Ergebnis!**

## Sie wollen gleich starten?

Das RoboCup-Team der TU Bergakademie sucht noch Mitstreiter. Auch wenn Sie noch keine Erfahrung in der Programmierung haben sind Sie gern willkommen.

![Atom IDE Screenshot](./img/00_Einfuehrung/RoboterNuernberg.jpg)<!-- width="100%" -->

## Autoren
Sebastian Zug, Georg Jäger, Hernán Valdés
