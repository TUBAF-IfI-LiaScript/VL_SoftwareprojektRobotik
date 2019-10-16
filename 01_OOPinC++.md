<!--

author:   Sebastian Zug & Georg Jäger
email:    sebastian.zug@informatik.tu-freiberg.de & Georg.Jaeger@informatik.tu-freiberg.de
version:  0.0.1
language: de
narrator: Deutsch Female

import: https://raw.githubusercontent.com/LiaTemplates/Rextester/master/README.md

-->

# Vorlesung II - Objektorientierte Programmierung in C++

Eine interaktive Version des Kurses finden Sie unter [Link](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/SoftwareprojektRobotik/master/01_OOPinC++.md#1)

**Zielstellung der heutigen Veranstaltung**


--------------------------------------------------------------------------------

## Von Strukturen und Klassen

Klassen und Strukturen unterscheiden sich unter C++ nur in einem Punkt. Während bei erstgenannten immer das Zugriffsattribut private als Default-Wert angenommen wird, ist dies für `struct`s public. Um eine Übersicht über die spezifischen Konzepte unter C++ wurde im folgenden auf structs abgezielt.

Eine Struktur (struct(ure)) ist ein Datentyp, der mehrere Variablen gleichen oder verschiedenen Typs zu einem neuen Datentyp zusammenfasst. Die Deklaration erfolgt mit dem Schlüsselwort `struct`.

```cpp                     ApplicationOfStructs.cpp
#include <iostream>

struct Student{
  std::string name;
  int alter;
  std::string ort;
};   // <- Dieses Semikoln wird gern vergessen :-)

int main()
{
  struct Student Alexander {"Humboldt", 23};
  std::cout << Alexander.ort;
}
```
@Rextester.CPP


| Umsetzung                                                                             | Beispiel                                                          |
|:--------------------------------------------------------------------------------------|:------------------------------------------------------------------|
| vollständige Liste in absteigender Folge (uniforme Initialisierung)                   | `Student Bernhard {"Cotta", 25, "Zillbach"};`                     |
| unvollständige Liste (die fehlenden Werte werden durch Standard Defaultwerte ersetzt) | `Student Bernhard {"Cotta", 25};`                                 |
| Elementinitialisierung nicht statischer Daten bei der Spezifikation (C++11)           | `struct Student{ std::string name = "unknown"; int alter = 18;};` |

Leider ist in C++11 die Elementinitialisierung nicht mit uniformen Initialisierungssyntax kompatibel und vermischbar. Folglich müssen Sie in C++11 entscheiden, ob Sie eine nicht statische Elementinitialisierung oder eine einheitliche Initialisierung verwenden möchten. In C++14 wird diese Einschränkung jedoch aufgehoben und beide können verwendet werden.

### Konstruktoren

Im Grunde können wir unsere drei Datenfelder im `struct` Student auf drei Arten
initialisieren:

```
{name, alter, ort}
{name, alter}
{name}
{}
```

Eine differenziertere Zuordnung der Reihenfolge `{name = "Zeuner", ort = "Chemnitz"}` unter Auslassung von Student.alter ist nicht möglich.
Diese Lücke wird durch Kontruktoren abgedeckt:

<!-- 1. Überladen von Funktionen wiederholen, Beispiele anhand unter-       -->
<!-- schiedlicher Parametersets                                             -->
<!--    Student::Student(std::string name) : name {name}, alter{18}, ort{"Freiberg"} -->

```cpp                     Constructors.cpp
#include <iostream>

struct Student{
  Student();
  std::string name;
  int alter;
  std::string ort;
  void printCertificate();
};

void Student::printCertificate(){
    if (name != "unknown"){
      std::cout << "Super," << this->name << " passed the exam!\n";
    }else{
      std::cout << "Student has not participated yet in a lecture!\n";
    }
}

Student::Student() : name {"unknown"}, alter {18}, ort {"Freiberg"}
{ }  // keine Funktionalität

int main()
{
  Student Erstsemester {};
  Erstsemester.printCertificate();
}
```
@Rextester.CPP

Delegierende Konstruktoren rufen einen weiteren Konstruktor für die teilweise
Initialisierung auf. Damit lassen sich Codedublikationen die sich aus der
Kombination aller Paramter ergeben zumindest minimieren.

```cpp
Student(string n, int a, string o): name{n}, alter{a}, ort{o} { }
Student(string n) : Student (n, 18, "Freiberg") {};
Student(int a, string o): Student ("unknown", a, o) {};
```

Einer Ihrer Kommilitonen kommt auf die Idee einer `init()` Methode, die die Initialsierung übernehmen soll. Was halten Sie von dieser Idee?

Copy-Konstruktoren gehen einen anderen Weg und aggregieren die Informationen unmittelbar aus einer bestehenden Instanz.

```cpp                     Constructors.cpp
#include <iostream>

struct Student{
  Student(const Student &oldStudent);
  Student(std::string n, int a, std::string o);
  std::string name;
  int alter;
  std::string ort;
};

Student::Student(std::string n, int a, std::string o): name{n}, alter{a}, ort{o} { }

Student::Student(const Student &oldStudent){
  this->name = oldStudent.name;
  this->alter = oldStudent.alter;
  this->ort = oldStudent.ort;
}

int main()
{
  Student Bernhard  {"Cotta", 27, "Freiberg"};
  Student Bernhard2(Bernhard);
  std::cout << Bernhard2.name << " " << Bernhard2.ort;
}
```
@Rextester.CPP

Nun drängt sich sofort die Frage auf, ob unsere beiden Bernhards identisch sind.
`if (Bernhard2 == Bernhard) {...}` ... Damit wären wir auch schon beim Thema Operatoren



### Methoden

<!-- 1. Einführung des this pointers                                        -->
<!-- 2. Expliziter Pointer auf Instanz (C)                                  -->
<!--      string generateText(Person * const p)                             -->

```cpp                     ApplicationOfStructs.cpp
#include <iostream>

struct Student{
  std::string name;
  int alter;
  std::string ort;
  void printCertificate();
};

void Student::printCertificate(){
    std::cout << "Super," << this->name << " passed the exam!\n";
}

int main()
{
  Student Alexander {"Humboldt", 23};
  Alexander.printCertificate();
}
```
@Rextester.CPP






### Operatoren

Welche Schwierigkeiten sehen Sie bei der Lösung? Was müssten sie tuen, wenn
Sie plötzlich für einen bestimmten Namen eine Ausgabe nicht auf den Bildschirm `std::cout` sondern nach `std::cerr` gelenkt werden soll?
