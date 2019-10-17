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

+ Das ist ein Test

--------------------------------------------------------------------------------

## Von Strukturen und Klassen

Klassen und Strukturen unterscheiden sich unter C++ nur in einem Punkt. Während
bei erstgenannten immer das Zugriffsattribut private als Default-Wert angenommen
wird, ist dies für `struct`s public. Um eine Übersicht über die spezifischen
Konzepte unter C++ wurde im folgenden auf structs abgezielt.

Eine Struktur (struct(ure)) ist ein Datentyp, der mehrere Variablen gleichen
oder verschiedenen Typs zu einem neuen Datentyp zusammenfasst. Die Deklaration
erfolgt mit dem Schlüsselwort `struct`.

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
| ------------------------------------------------------------------------------------- | ----------------------------------------------------------------- |
| vollständige Liste in absteigender Folge (uniforme Initialisierung)                   | `Student Bernhard {"Cotta", 25, "Zillbach"};`                     |
| unvollständige Liste (die fehlenden Werte werden durch Standard Defaultwerte ersetzt) | `Student Bernhard {"Cotta", 25};`                                 |
| Elementinitialisierung nicht statischer Daten bei der Spezifikation (C++11)           | `struct Student{ std::string name = "unknown"; int alter = 18;};` |

Leider ist in C++11 die Elementinitialisierung nicht mit uniformen Initialisierungssyntax kompatibel und vermischbar. Folglich müssen Sie in C++11 entscheiden, ob Sie eine nicht statische Elementinitialisierung oder eine einheitliche Initialisierung verwenden möchten. In C++14 wird diese Einschränkung jedoch aufgehoben und beide können verwendet werden.

### Konstruktoren
<!--
  comment: Constructor.cpp
  ..............................................................................
    1. Überladen von Funktionen wiederholen,
       Beispiele für den Konstruktor anhand unterschiedlicher Parametersets          
       ```cpp                                
        Student::Student(std::string name) : name {name}, alter{18}, ort{"Freiberg"};
       ```
    2. Ein Beispiel für Delegation einfügen

  comment: CopyConstructor.cpp
  ..............................................................................
      1. Was fällt Ihnen auf? Das Copy Assignment wird gar nicht ausgeführt!
         Warum nicht?       
         ```cpp
           Student gustav3 {"Anton"};
           gustav3 = gustav;
         ```
-->

                            {{0-1}}
********************************************************************************

Im Grunde können wir unsere drei Datenfelder im vorangegangen Beispiel in vier
Kombinationen  initialisieren:

```
{name, alter, ort}
{name, alter}
{name}
{}
```

Eine differenziertere Zuordnung der Reihenfolge `{name = "Zeuner", ort =
"Chemnitz"}` unter Auslassung von Student.alter ist nicht möglich.

Was passiert aber bei dem Aufruf? Der Kompiler generiert uns automatisch einen
Standardkonstruktor, wenn Sie gar keinen eigenen Konstruktor generiert haben.

<!--Ist es wirklich EIN Standardkonstruktor? immerhin habe ich ja 4 Signaturen?-->


********************************************************************************

                          {{1-2}}
********************************************************************************

Welche Varianten sind für die Erzeugung einer Instanz denkbar, sprich wie kann
ich individuelle Mechanismen für die Intialisierung eines Objektes definieren?

 1. Erzeugung auf der Basis eines Parametersets mit individuellem Konstruktor

```cpp  
   Student Erstsemester(int alter);
```

 2. Erzeugung auf der Basis einer existierenden Instanz, die als Parameter übergeben wird (copy constructor)

```cpp  
   Student ErstsemesterTemplate();
   Student Erstsemester(ErstsemesterTemplate);
```

 3. Erzeugung auf der Basis einer existierenden Instanz, per Verschiebung (move constructor)

```cpp  
  Student ErstsemesterTemplate();
  Student Erstsemester ????
```
********************************************************************************

   {{2-3}}
********************************************************************************

**Nicht-Default Basiskonstruktoren**

```cpp                     Constructor.cpp
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

Einer Ihrer Kommilitonen kommt auf die Idee einer `init()` Methode, die die
Initialsierung übernehmen soll. Was halten Sie von dieser Idee?

********************************************************************************


   {{3-4}}
********************************************************************************

**Copy-Konstruktoren**

Copy-Konstruktoren gehen einen anderen Weg und aggregieren die Informationen
unmittelbar aus einer bestehenden Instanz. Welche Schritte sind im folgenden
Beispiel notwendig, um einen Studenten aus der Bachelorliste in die Masterliste
zu transferieren. Logischerweise sollte der Student dann nur noch in der
Masterliste enthalten sein! Intuitiv würde diese bedeuten:

1. Erzeugen einer neuen Instanz von `Student` und initialisieren mit einer Kopie des existierenden Studenten
2. Löschen des Studenten in der Bachelorliste

Auf diesem Weg bestehen zwischen 1 und 2 letztendlich zwei Kopien des Studenten,
was für aufwändigere Datentypen (Bilder, Messungen) zu vermeiden ist!

```cpp                     CopyConstructor.cpp
#include <iostream>
#include <list>
#include <iterator>

struct Student{
  Student(const Student&);
  Student(std::string n);
  Student(std::string n, int a, std::string o);
  Student& operator=(const Student&);
  std::string name;
  int alter;
  std::string ort;
};

Student::Student(std::string n, int a, std::string o): name{n}, alter{a}, ort{o} {}

// Copy-Constructor
Student::Student(const Student& other){
  std::cout << "Copy constructor executed!\n";
  this->name = other.name;
  this->alter = other.alter;
  this->ort = other.ort;
}

void showlist(std::list <Student> g)
{
    for(auto it = g.begin(); it != g.end(); ++it)
        std::cout << it->name << "\n";
    std::cout << '\n';
}

int main()
{
  std::list <Student> Bachelor, Master;
  Student max {"Maier", 19, "Dresden"};
  Bachelor.push_back(max);
  Student gustav {"Zeuner", 27, "Chemnitz"};
  Master.push_back(gustav);
  showlist(Bachelor);
  showlist(Master);
}
```
@Rextester.CPP

Der Verschiebungskonstruktor löst dieses Problem.

```cpp
Student::Student(Student&& other){
  // leeres Objekt erzeugen
  // ???
}
```

Stop! Was bedeutet das &&?

********************************************************************************

### Destruktoren

```cpp                     CopyConstructor.cpp
#include <iostream>

struct Student{
  Student(std::string n, int a, std::string o);
  ~Student();
  Student& operator=(const Student&);
  std::string name;
  int alter;
  std::string ort;
};

Student::Student(std::string n, int a, std::string o): name{n}, alter{a}, ort{o} {}

Student::~Student(){
  std::cout << "Destructor executed \n";
}

int main()
{
  Student max {"Maier", 19, "Dresden"};
  return 0;
}
```
@Rextester.CPP

Destruktoren werden aufgerufen, wenn eines der folgenden Ereignisse eintritt:

* Das Programm verlässt den Gültigkeitsbereich eines lokalen Objektes.
* Ein Objekt, das `new`-erzeugt wurde, wird mithilfe von `delete` explizit aufgehoben (Speicherung auf dem Heap)
* Ein Programm endet und es sind globale oder statische Objekte vorhanden.
* Der Destruktor wird unter Verwendung des vollqualifizierten Namens der Funktion explizit aufgerufen.

Einen Destruktor explizit aufzurufen, ist selten notwendig.

> **Merke:** Ein Destruktor darf keine Exception auslösen!

### Operatoren
<!--
  comment: Comparison.cpp
  ..............................................................................
      1. std::sort greift bei den Objekten auf die "<" Operation zurück.
         Fügen Sie eine Operatorüberladung für "<" ein und sortieren Sie die Liste.    
         ```cpp
         bool Student::operator==(const Student& other){
           if (this->name > other.name) {
             return true;
           }else{
             return false;
           }
         }
         ```
  comment: Comparison.cpp
  ..............................................................................
     1. Erläuterung zum Unterschied zwischen Zuweisung und Initiierung    
        ```cpp
        Student gustav3;
        gustav3 = gustav2;
        ```
-->

                 {{0-1}}
********************************************************************************

Mit dem Überladen von Operatoren \+, \-, \*, \/, = kann deren Bedeutung für
selbstdefinierter Klassen (fast) mit einer neuen Bedeutung versehen werden.
Diese Bedeutung wird durch spezielle Funktionen bzw. Methoden festgelegt.

Im folgenden Beispiel wird der Vergleichsoperator `==` überladen. Dabei sehen
wir den Abgleich des Namens und des Alters als ausreichend an.

**Allgemeine Operatoren**

```cpp                     Comparison.cpp
#include <iostream>
#include <vector>
#include <algorithm>

struct Student{
  Student(const Student&);
  Student(std::string n);
  Student(std::string n, int a, std::string o);
  bool operator==(const Student&);
  std::string name;
  int alter;
  std::string ort;
};

Student::Student(std::string n): name{n}, alter{18}, ort{"Freiberg"}{}

Student::Student(std::string n, int a, std::string o): name{n}, alter{a}, ort{o} {
}

Student::Student(const Student& other){
  this->name = other.name;
  this->alter = other.alter;
  this->ort = other.ort;
}

bool Student::operator==(const Student& other){
  if ((this->name == other.name) && (this->alter == other.alter)){
    return true;
  }else{
    return false;
  }
}

int main()
{
  Student gustav {"Zeuner", 27, "Chemnitz"};
  Student alexander {"Humboldt", 22, "Berlin"};
  Student bernhard {"Cotta", 18, "Zillbach"};
  Student gustav2(gustav);              
  if (gustav == gustav2){
    std::cout << "Identische Studenten \n";
  }else{
    std::cout << "Ungleiche Identitäten \n";
  }
  std::vector<Student> StudentList {gustav, alexander, bernhard, gustav2};
  for (auto &i: StudentList)
      std::cout << i.name << ", ";
  std::cout << std::endl;

  //std::sort(StudentList.begin(), StudentList.end());
  //for (auto &i: StudentList)
  //    std::cout << i.name << ", ";
  //std::cout << std::endl;

}
```
@Rextester.CPP

Mit der Operatorüberladung von `>` haben wir ein Sortierkriterium abgebildet.
Wie würden Sie vorgehen, wenn sich Ihr Auftraggeber hier eine größere Flexibilität wünscht und ein Set von Metriken bereit gehalten werden soll?

********************************************************************************

                                 {{1-2}}
********************************************************************************

**Zuweisungsoperatoren**

Zuweisungsoperatoren können in zwei Konfigurationen realisiert werden.

* Kopierend ... die auf der rechten Seite stehende Instanz bleibt erhalten, so dass nach der Operation zwei Objekte bestehen (copy assignment)
* Verschiebend ... die auf der rechten Seite stehend Instanz wird kopiert, so dass nur die linke Instanz weiter besteht (move assignment)

Die Unterscheidung der Mechanismen erfolgt anhand der Signaturen der Operatorüberladungen:

```cpp  
// copy assignment
myClass& operator=(const myClass& other)
{
  if(this != &other) {
     return *this = myClass(other);      // Aufruf des Copy Constructors
  }
}

// move assignment
myClass& operator=(const myClass&& other)
{
    if(this != &other) { //1. Überprüfung der Zuweisung auf sich selbst
      // ...               2. Freigeben von bisherigen Ressourcen
      // ...               3. elementweises Übertragen
    }
    return *this;       // 4. Rückgabe der
}
```

Im Beispiel hat einer Ihrer Kommilitonen das Copy-Assignment implementiert. Die
Lösung generiert aber eine unerwartete Ausgabe. Welche Irrtum ist der Kandidat
erlegen?

```cpp                     Assignment.cpp
#include <iostream>

struct Student{
  Student(const Student&);
  Student(std::string n);
  Student(std::string n, int a, std::string o);
  Student& operator=(const Student&);
  std::string name;
  int alter;
  std::string ort;
};

Student::Student(std::string n): name{n}, alter{18}, ort{"Freiberg"}{}

Student::Student(std::string n, int a, std::string o): name{n}, alter{a}, ort{o} {
  std::cout << "Non-default constructor executed!\n";
}

// Copy-Constructor
Student::Student(const Student& other){
  std::cout << "Copy constructor executed!\n";
  this->name = other.name;
  this->alter = other.alter;
  this->ort = other.ort;
}

// Copy Assignment
Student& Student::operator=(const Student& other){
  std::cout << "Copy assignment executed!\n";
  if(&other != this){          
    *this = Student(other);  
  }
  return *this;                
}

int main()
{
  Student gustav {"Zeuner", 27, "Chemnitz"};
  Student gustav2(gustav);
  Student gustav3 = gustav;
  gustav.alter = 29;           // Hiermit wird geprüft ob eine unabhängige Kopien
                               // von gustav entstanden sind.
  std::cout << gustav2.name << " " << gustav2.alter << "\n";
  std::cout << gustav3.name << " " << gustav3.alter << "\n";
}
```
@Rextester.CPP

> **Merke**: Ein Gleichheitszeichen in einer Variablendeklaration ist niemals
> eine Zuweisung, sondern nur eine andere Schreibweise der Initialisierung! (vgl. Zeile 41 im vorangegangen Codebeispiel)

********************************************************************************

## Rule of Five

Der Kompiler generiert automatisch für Sie:

+ einen Standardkonstruktor (wenn Sie gar keinen Konstruktor selbst angelegt haben)
+ einen Destruktor
+ einen Kopierkonstruktor
+ einen Kopierzuweisung
+ den Verschiebekonstruktor (seit C++11)
+ den Verschiebeoperator (seit C++11)

Die generierten Versionen haben dabei eine in der Sprachnorm festgelegte Bedeutung: Es werden alle nicht-statischen Datenelemente in der Reihenfolge ihrer Deklaration kopiert (Konstruktoren bzw. Zuweisungen) bzw. in umgekehrter Reihenfolge freigegeben (Destruktor).

Super! Alles gelöst! Warum müssen wir also darüber nachdenken?

                                 {{1-2}}
********************************************************************************

Falls eine Klasse jedoch eine andere Semantik hat, z. B. weil sie eine Ressource als Datenelement enthält, die nicht auf diese Weise kopiert oder abgeräumt werden kann, kann jede der genannten Konstruktoren/Destruktoren/Operatoren durch eine eigene Definition ersetzt werden. In den meisten Fällen erfordern solche Klassen dann, dass alle  eigene, benutzerdefinierte Implementierung benötigen.

Beispiel:

```cpp
class Datei
{
public:
    Datei(const char* dateiname)
    : file(fopen(dateiname, "rb"))
    { /* Fehlerbehandlung usw. */ }

    // Dreierregel:
    Datei(const Datei&) = delete; // Kein Kopieren!
    ~Datei() { fclose(file); }
    void operator=(const Datei&) = delete; // Kein Kopieren!

    // weitere Elementfunktionen
    // ...

private:
    FILE* file;
};
```

Seit C++11 ist es zudem möglich, das Erzeugen der compilergenerierten Version nicht nur explizit zu unterdrücken, sondern auch explizit zu erzwingen (=default). Damit wird dem Compiler (und auch dem menschlichen Leser) mitgeteilt, das in diesem Fall die compilergenerierte Version genau das gewünschte Verhalten bietet, so dass man es nicht manuell implementieren muss:

```cpp
class Example
{
    Example(const Example&) = default;  // erzwinge compilergenerierte Version
    void operator=(const Example&) = delete; // verhindere compilergenerierte Version
};
```

********************************************************************************


## Aufgabe der Woche

1. Implementieren Sie das Move Assignment Operation in Beispiel Assignment.cpp
