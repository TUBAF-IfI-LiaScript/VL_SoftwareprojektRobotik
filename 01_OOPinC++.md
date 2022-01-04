<!--

author:   Sebastian Zug & Georg Jäger
email:    sebastian.zug@informatik.tu-freiberg.de & Georg.Jaeger@informatik.tu-freiberg.de
version:  0.0.1
language: de
narrator: Deutsch Female

import:   https://github.com/liascript/CodeRunner

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/VL_SoftwareprojektRobotik/master/01_OOPinC++.md#1)

# Objektorientierte Programmierung in C++

| Parameter            | Kursinformationen                                                                                                                                                                             |
| -------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Veranstaltung:**   | `Softwareprojekt Robotik`                                                                                                                                                                     |
| **Semester**         | `Wintersemester 2021/22`                                                                                                                                                                      |
| **Hochschule:**      | `Technische Universität Freiberg`                                                                                                                                                             |
| **Inhalte:**         | `Konstruktoren und Operatoren`                                                                                                                                                |
| **Link auf GitHub:** | [https://github.com/TUBAF-IfI-LiaScript/VL_Softwareentwicklung/blob/master/00_Einfuehrung.md](https://github.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/blob/master/01_OOPinC++.md) |
| **Autoren**          | @author                                                                                                                                                                                       |

![](https://media.giphy.com/media/l46CwEYnbFtFfjZNS/giphy-downsized.gif)

--------------------------------------------------------------------------------

**Zielstellung der heutigen Veranstaltung**

+ Klassen/Strukturen
+ Rule of Five
+ Operatoren

--------------------------------------------------------------------------------

## Von Strukturen und Klassen

Klassen (`class`) und Strukturen (`struct`) unterscheiden sich unter C++ nur in einem Punkt. Während
bei erstgenannten immer das Zugriffsattribut private als Default-Wert angenommen
wird, ist dies für `struct`s public. Die folgenden Beispiele nutzen structs um C++ spezifische Eigenschaften darzustellen, können aber direkt auf Klassen übertragen werden.

Eine Struktur ist ein Datentyp, der mehrere Variablen gleichen
oder verschiedenen Typs zu einem neuen Datentyp zusammenfasst. Die Deklaration
erfolgt mit dem Schlüsselwort `struct`.

```cpp                     ApplicationOfStructs.cpp
#include <iostream>

struct Student{
  std::string name;
  int alter;
  std::string ort;
};   // <- Dieses Semikolon wird gern vergessen :-)

int main()
{
  Student bernhard = {"Cotta", 25, "Zillbach"};
  std::cout << bernhard.ort << " " << bernhard.alter  << std::endl;
  //Student alexander = { .name = "Humboldt" , .alter = 22 , .ort = "Berlin"  };
  //std::cout << alexander.ort << " " << alexander.alter  << std::endl;
  return EXIT_SUCCESS;
}
```
@LIA.evalWithDebug(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

**Elementinitialisierung:**

| Umsetzung                                                                             | Beispiel                                      |
|:------------------------------------------------------------------------------------- |:--------------------------------------------- |
| vollständige Liste in absteigender Folge (uniforme Initialisierung)                   | `Student Bernhard {"Cotta", 25, "Zillbach"};` |
| unvollständige Liste (die fehlenden Werte werden durch Standard Defaultwerte ersetzt) | `Student Bernhard {"Cotta", 25};`             |
| vollständig leere Liste, die zum Setzen von Defaultwerten führt                                                                                      | `Student Bernhard {};`                        |
| Aggregierende Initialisierung (C++20)                                                                                      |   `Student alexander = { .ort = "unknown"}; `                                             |

C++11 führte die uniformen Initialisierungssyntax ein. In C++20 wird dieser Mechanismus um die die _Designated Initialisers_, die Sie aus C kennen erweitert.

Wo ist bei den _Designated Initialisers_ der Unterschied zu C? Die C++ Implementierung integriert nicht:

+ eine variable Reihenfolge der Member zu initialisieren.
+ die Member eines verschachtelten Aggregates zu initialisieren.
+ Designated Initializers und reguläre Initialisierer zu vermischen.


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
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  comment: CopyConstructor.cpp
  ..............................................................................
      1. Was fällt Ihnen auf? Das Copy Assignment wird gar nicht ausgeführt!
         Warum nicht?
         ```cpp
         // Assignment Operator
          Student& Student::operator=(const Student& other){
          	if (this != &other){
          		  std::cout << "Assignment operator executed!\n";
          		  this->name = other.name;
          		  this->alter = other.alter;
          		  this->ort = other.ort;
          	}
          }
         ```
      2. Erweitern des Beispiels um Move-Constructors und deren Einsatz mittels std::move
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
"Chemnitz"}` unter Auslassung von Student.alter ist nur möglich, wenn hierfür ein default Initializer vorgesehen ist.

Was passiert aber bei dem Aufruf `Student alexander {"Humboldt", 23};`? Der Kompiler generiert uns implizit/automatisch passende
Konstruktor(en), wenn Sie gar keinen eigenen Konstruktor generiert haben. Diese werden daher auch *Implicit Constructors* genannt.

********************************************************************************

                          {{1-2}}
********************************************************************************

Welche Varianten sind für die Erzeugung einer Instanz denkbar, sprich wie kann
ich individuelle Mechanismen für die Intialisierung eines Objektes definieren?

 1. Erzeugung auf der Basis eines Parametersets mit individuellem Konstruktor

```cpp
   int alter = 21;
   Student erstsemester(alter);
```

 2. Erzeugung auf der Basis einer existierenden Instanz, die als Parameter übergeben wird (*Copy Constructor*)

```cpp
   Student erstsemester_template();
   Student erstsemester(erstsemester_template);
```

 3. Erzeugung auf der Basis einer existierenden Instanz, per Verschiebung (*Move Constructor*)

```cpp
  Student erstsemester_template();
  ...
  Student erstsemester (std::move(erstsemester_template));
```
********************************************************************************

   {{2-3}}
********************************************************************************

**Nicht-Default Basiskonstruktoren**

```cpp                     Constructor.cpp
#include <iostream>

struct Student{
  std::string name;
  int alter;
  std::string ort;

  Student(); // Default Constructor
  void printCertificate();
};

void Student::printCertificate(){
	  std::cout << "Student " << this->name << " " << this->alter << " passed the exam!\n";
}

Student::Student() : name {"Cotta"}, alter {18}, ort {"Freiberg"}
{ }  // keine Funktionalität

int main()
{
  Student erstsemester {};
  erstsemester.printCertificate();
}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

Delegierende Konstruktoren rufen einen weiteren Konstruktor für die teilweise
Initialisierung auf. Damit lassen sich Codeduplikationen, die sich aus der
Kombination aller Paramter ergeben, minimieren.

```cpp
Student(std::string n, int a, std::string o): name{n}, alter{a}, ort{o} { }
Student(std::string n) : Student (n, 18, "Freiberg") {};
Student(int a, std::string o): Student ("unknown", a, o) {};
```

Einer Ihrer Kommilitonen kommt auf die Idee einer `init()` Methode, die die
Initialsierung übernehmen soll. Was halten Sie von dieser Idee?

********************************************************************************


   {{3-4}}
********************************************************************************

**Copy-Konstruktoren**

Copy-Konstruktoren gehen einen anderen Weg und aggregieren die Informationen
unmittelbar aus einer bestehenden Instanz.

Welche Schritte sind im folgenden
Beispiel notwendig, um einen Studenten aus der Bachelorliste in die Masterliste
zu transferieren? Logischerweise sollte der Student dann nur noch in der
Masterliste enthalten sein! Intuitiv würde dies bedeuten:

1. Erzeugen einer neuen Instanz von `Student` und initialisieren mit einer Kopie des existierenden Studenten
2. Löschen des Studenten in der Bachelorliste

Auf diesem Weg bestehen zwischen 1 und 2 letztendlich zwei Kopien des Studenten,
was für aufwändigere Datentypen (Bilder, Messungen) zu vermeiden ist!

```cpp                     CopyConstructor.cpp
#include <iostream>
#include <list>
#include <iterator>

struct Student{
  std::string name;
  int alter;
  std::string ort;

  Student(std::string n, int a, std::string o);

  Student(const Student&);               // Copy Constructor
  //Student& operator=(const Student&);    // Copy Alignment
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
  Student max {"Maier", 19, "Dresden"};
  Student gustav {"Zeuner", 27, "Chemnitz"};
  Student x = gustav;                   // initialization by copy constructor
  Student y(max);                       // Also initialization by copy constructor
  //y = gustav;                           // assignment by copy assignment operator
  //std::cout  << y.ort;
  //std::list <Student> bachelor, master;
  //bachelor.push_back(max);
  //master.push_back(gustav);
  //showlist(bachelor);
  //showlist(master);
}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

Der Verschiebungskonstruktor löst dieses Problem.

```cpp
Student::Student(Student&& other) noexcept {
  this->name = std::move(other.name);
  this->alter = other.alter;
  this->ort = std::move(other.ort);
}
```

Während das '&' eine Variable als Referenz deklariert, legt '&&' eine 'rvalue-Referenz' an. D.h. eine Referenz auf ein Objekt, dessen Lebensdauer *am Ende ist*. (Mehr dazu später...)

********************************************************************************

### Destruktoren
<!--
  comment: Destructor.cpp
  .............................................................................
        - Show with different scopes {}
        - Show with new/delete
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-->

```cpp                     Destructor.cpp
#include <iostream>

struct Student{
  std::string name;
  int alter;
  std::string ort;

  Student(std::string n, int a, std::string o);
  ~Student();
};

Student::Student(std::string n, int a, std::string o): name{n}, alter{a}, ort{o} {}

Student::~Student(){
  std::cout << "Destructing object of type 'Student' with name = '" << this->name << "'\n";
}

int main()
{
  Student max {"Maier", 19, "Dresden"};
  std::cout << "End...\n";
  return 0;
}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

Destruktoren werden aufgerufen, wenn eines der folgenden Ereignisse eintritt:

* Das Programm verlässt den Gültigkeitsbereich (*Scope*, d.h. einen Bereich der mit `{...}` umschlossen ist) eines lokalen Objektes.
* Ein Objekt, das `new`-erzeugt wurde, wird mithilfe von `delete` explizit aufgehoben (Speicherung auf dem Heap)
* Ein Programm endet und es sind globale oder statische Objekte vorhanden.
* Der Destruktor wird unter Verwendung des vollqualifizierten Namens der Funktion explizit aufgerufen.

Einen Destruktor explizit aufzurufen, ist selten notwendig (oder gar eine gute Idee!).

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
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  comment: Assignment.cpp
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
  std::string name;
  int alter;
  std::string ort;

  Student(const Student&);
  Student(std::string n);
  Student(std::string n, int a, std::string o);

  bool operator==(const Student&);
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
  std::vector<Student> studentList {gustav, alexander, bernhard, gustav2};
  for (auto &i: studentList)
      std::cout << i.name << ", ";
  std::cout << std::endl;

  //std::sort(studentList.begin(), studentList.end());
  //for (auto &i: studentList)
  //    std::cout << i.name << ", ";
  //std::cout << std::endl;

}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

Analysieren Sie die Hinweise zur Sortiermethode in der Standard-Bibliothek

https://en.cppreference.com/w/cpp/algorithm/sort

Wie kann die entsprechende Sortierfunktion übergeben werden? Wann kann darauf
verzichtet werden?

Mit der Operatorüberladung von `<` haben wir ein Sortierkriterium abgebildet.
Wie würden Sie vorgehen, wenn sich Ihr Auftraggeber hier eine größere Flexibilität wünscht und ein Set von Metriken bereit gehalten werden soll?

********************************************************************************

                                 {{1-2}}
********************************************************************************

**Zuweisungsoperatoren**

Zuweisungsoperatoren können in zwei Konfigurationen realisiert werden.

* Kopierend ... die auf der rechten Seite stehende Instanz bleibt erhalten, so dass nach der Operation zwei Objekte bestehen (*Copy Assignment*)
* Verschiebend ... die auf der rechten Seite stehend Instanz wird kopiert, so dass nur die linke Instanz weiter besteht (*Move Assignment*)

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
myClass& operator=(myClass&& other) noexcept
{
    if(this != &other) { //1. Überprüfung der Zuweisung auf sich selbst
      // ...               2. Freigeben von bisherigen Ressourcen
      // ...               3. elementweises Übertragen
    }
    return *this;       // 4. Rückgabe der
}
```

Im Beispiel hat einer Ihrer Kommilitonen das Copy-Assignment implementiert. Die
Lösung generiert aber eine unerwartete Ausgabe. Welchem Irrtum ist der Kandidat
erlegen?

```cpp                     Assignment.cpp
#include <iostream>

struct Student{
  std::string name;
  int alter;
  std::string ort;

  Student(const Student&);
  Student(std::string n);
  Student(std::string n, int a, std::string o);
  Student& operator=(const Student&);
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
    this->name = other.name;
    this->alter = other.alter;
    this->ort = other.ort;
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
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

> **Merke**: Ein Gleichheitszeichen in einer Variablendeklaration ist niemals
> eine Zuweisung, sondern nur eine andere Schreibweise der Initialisierung! (vgl. Zeile 41 im vorangegangen Codebeispiel)

********************************************************************************

## Rule of Five

Der Kompiler generiert automatisch für Sie:

+ einen Standardkonstruktor (wenn Sie gar keinen Konstruktor selbst angelegt haben)
+ einen Destruktor
+ einen Kopierkonstruktor
+ einen Kopierzuweisungsoperator
+ den Verschiebekonstruktor (seit C++11)
+ den Verschiebeoperator (seit C++11)

Die generierten Versionen haben dabei eine in der Sprachnorm festgelegte Bedeutung: Es werden alle nicht-statischen Datenelemente in der Reihenfolge ihrer Deklaration kopiert (Konstruktoren bzw. Zuweisungen) bzw. in umgekehrter Reihenfolge freigegeben (Destruktor).

Super! Alles gelöst! Warum müssen wir also darüber nachdenken?

                                 {{1-2}}
********************************************************************************

Falls eine Klasse jedoch eine andere Semantik hat, z. B. weil sie eine Ressource als Datenelement enthält, die nicht auf diese Weise kopiert oder abgeräumt werden kann, kann jede der genannten Konstruktoren/Destruktoren/Operatoren durch eine eigene Definition ersetzt werden. In den meisten Fällen erfordern solche Klassen dann, dass alle Konstruktoren/Destruktoren/Operatoren eigene, benutzerdefinierte Implementierungen haben.

Beispiel:

```cpp
class Datei
{
public:
    Datei(const char* dateiname)
    : file(fopen(dateiname, "rb"))
    { /* Fehlerbehandlung usw. */ }

    // Rule of Three (bis C++11):
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

## Wiederholung - Gültigkeitsbereich von Variablen

Wie lange ist meine Variable, die ich deklariert und initialisiert habe, verfügbar?

C++ definiert dafür 5 Gültigkeitsbereiche einer Variable, die einem jederzeit
bewusst sein sollten, um "Irritationen" zu vermeiden:

* Globalen Gültigkeitsbereich - eine Variable oder ein Objekt wird außerhalb von jeder Klasse, Funktion oder Namespace deklariert. C++ ordnet diese automatisch einem globalen Namespace zu.

```cpp                    globalVariables.cpp
#include <iostream>

int i = 8;   // i has global scope, outside all blocks
int j = 5;

int main( int argc, char *argv[] ) {
   int i = 4;   // das lokale i verdeckt das globale
   std::cout << "Block scope der Variable    : " << i << "\n";
   std::cout << "Global scope der Variable i : " << ::i << "\n";
   std::cout << "Global scope der Variable j : " << j << "\n";
}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

* Namespace-Gültigkeitsbereiche - moduluarisiert Projekte, in die einzelenen Bestandteile individuell gekapselt werden. Variabeln, die innerhalb eines Namespaces "global" angelegt wurden sind nur in diesem sichtbar. Ein Namespace kann in mehreren Blöcken in verschiedene Dateien definiert werden.

```cpp                    namespaces.cpp
#include <iostream>

int global = 10;

namespace myFunction{
  int global = 5;

  void doubleGlobal(){
    global += global;
  }
}

int main( int argc, char *argv[] ) {
   std::cout << "Block scope der Variable global         : " << global << "\n";
   std::cout << "Variable global im namespace myFunction : " << myFunction::global << "\n";
}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

* Lokaler Gültigkeitsbereich - Variablen oder Objekte, die innerhalb eines Anweisungsblock oder einem Lambda-Ausdrucks deklariert werden, haben lokale Gültigkeit. Alle Formen von `{ // Anweisungen}` definieren dabei einen eigenen Block (oder *scope*).

```cpp                    localVariables.cpp
#include <iostream>

void myFunction(){
  int i = 10;
  std::cout << "Variable i im eingebetteten scope   : " << i << "\n";
}

int main( int argc, char *argv[] ) {
   int i = 0;
   std::cout << "Variable i im aktuellen scope      : " << i << "\n";
   {
     int i = 5;
     std::cout << "Variable i im Scope der Funktion : " << i << "\n";
   }
   std::cout << "Variable i im aktuellen scope      : " << i << "\n";
   myFunction();
}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

* Anweisungsbereichs - Anweisungsblöcke erweitern das Konzept des Anweisungsbereiches um die Parameter der Anweisung.

```cpp                    localVariables.cpp
#include <iostream>

int main( int argc, char *argv[] ) {
  for (auto i = 0; i<10; i++){
    int j = 5;
    result += i;
  }
  std::cout << "Das Ergebnis für " << i << " Schleifen lautet " << result << "\n";
}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

* Klassengültigkeitsbereich - Membervariablen oder Funktionen, die im im Definitionsbereich der Klasse liegen, können nur über die entsprechenden Instanzen oder ggf. als statische Klassenelemente über den Klassennamen adressiert werden. Weiter gesteuert wird dieser Zugriff über öffentliche, private, und geschützt Schlüsselwörter.

```cpp                    classMember.cpp
#include <iostream>
#include <string>

struct Student{
  std::string name;
};

int main()
{
  Student erstsemester {"Gustav"};
  std::cout << "Membervariable 'name' of 'erstsemester' has value '" << erstsemester.name << "'\n";
}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

## Aufgabe der Woche

1. Implementieren Sie die Move Assignment Operation in Beispiel Assignment.cpp
2. Implementieren Sie eine Klasse, die Lese-/Schreiboperationen für Sie realisiert. Warum ist es gerade hier notwendig die Rule-of-Five Idee zu berücksichtigen?
