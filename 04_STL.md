<!--

author:   Sebastian Zug & Georg Jäger
email:    sebastian.zug@informatik.tu-freiberg.de & Georg.Jaeger@informatik.tu-freiberg.de
version:  0.0.1
language: de
narrator: Deutsch Female

import: https://raw.githubusercontent.com/LiaTemplates/Rextester/master/README.md

-->

# Vorlesung IV - STL

Eine interaktive Version des Kurses finden Sie unter [Link](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/SoftwareprojektRobotik/master/04_STL.md#1)

**Zielstellung der heutigen Veranstaltung**

+ Elemente effektiver Softwareentwicklung unter C++
+ knappe Einführung in die Container der Standardbibliothek
+ Überblick zu den Algorithmen über den Containertypen

--------------------------------------------------------------------------------

## Nehmen wir mal an ...

... sie wollen ein Projekt in C++ realisieren. Welche Unterstützung haben Sie von Seiten der Sprache, um einen wartbaren und übertragbaren Code zu definieren. Welchen Implementierungsmustern sollen Sie folgen:

+ Verwendung der Standardbibliothek (kleiner Maßstab)
+ Implementierung von Designpatterns (mittlere Ebene)
+ externe Bibliotheken
+ Realisierung von existierenden Architekturkonzepten

## STL Kurzeinführung
<!--
  comment: rawarray.cpp
  ..............................................................................
  1. Wir verlieren bei der Übergabe des Arrays dessen Größeninformation
  ```cpp
  void print(Student studlist[]){
  	for(unsigned int i = 0; i < sizeof(studlist) / sizeof(class Student); i++)
    {
        std::cout << &studlist[i] << " - " << studlist[i] << std::endl;
    }
  }
  ```
  Der Aufruf generiert ein Warning, das darauf hinweist, das sizeof(studlist)
  nunmehr die Größe des Pointers auf den ersten Eintrag bereitstellt.
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  comment: stdarray.cpp
  ..............................................................................
  1. Für viele der Aufgaben im Umgang mit Containern bestehen bereits
  Implementierungen. Nehmen wir also an, dass wir einen neuen Container
  vollständig mit einem Wert befüllen wollen
  ```cpp
  std::array<int,6> myarray;
  myarray.fill(5);
  for ( int& x : myarray) { std::cout << ' ' << x; }
  ```
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-->

Ausgangspunkt ist die Behandlung von Datensammlungen, wie Sie sie aus C kennen. Wir erzeugen ein Array, innerhalb dessen die Datenstrukturen sequential abgelegt werden.

```cpp                      rawarray.cpp
#include <iostream>

class Student{
  private:
     int matrikel;
     std::string name;
  public:
     Student(int matrikel, std::string name): matrikel(matrikel), name(name) {};
     friend std::ostream& operator<<(std::ostream& os, const Student& s);
};

std::ostream& operator<<(std::ostream& os,  const Student& s){  
   os << s.name << "," << s.matrikel;
 return os;
}

int main()
{
  const int size = 3;
  Student studlist[] = {Student(123, "Alexander"),
                         Student(345, "Julius"),
                         Student(789, "Karl")};

  // Iteration mittels Indexvariable
  for(unsigned int i = 0; i < sizeof(studlist) / sizeof(class Student); i++)
  {
      std::cout << &studlist[i] << " - " << studlist[i] << std::endl;
  }

  std::cout << std::endl;

  // Iteration über Größeninforamtion
  for (Student* i = studlist; i <= &studlist[size-1]; i++){             
     std::cout << i << " - " <<  *i << std::endl;
  }

  return EXIT_SUCCESS;
}
```
@Rextester.CPP

**Welche Nachteile ergeben sich aus diesem Lösungsansatz?**

Wenn das Array an eine Funktion übergeben wird, verlieren wir die Information
über seine Größe! Es fehlt letztendlich eine Managementebene für die Datensammlung. Zudem ist die statische Konfiguration in vielen Fällen hinderlich.

Lösungsansatz 1: Wir implementieren uns jeweils eine Management-Klasse, die die Aggregation des Speichers, die aktuelle Größe usw. für uns koordiniert.

Lösungsansatz 2: Wir verwenden die Standardbibliothek, die unter anderem generische Containertypen bereithält.

```cpp                      stdarray.cpp
#include <iostream>
#include <array>

class Student{
  private:
     int matrikel;
     std::string name;
  public:
     Student(int matrikel, std::string name): matrikel(matrikel), name(name) {};
     friend std::ostream& operator<<(std::ostream& os, const Student& s);
};

std::ostream& operator<<(std::ostream& os,  const Student& s){  
   os << s.name << "," << s.matrikel;
 return os;
}

int main()
{
  const int size = 3;
  std::array<Student, size> studlist = {Student(123, "Alexander"),
                                        Student(345, "Julius"),
                                        Student(789, "Karl")};

  std::cout << studlist.size() << " Elements in array" << std::endl;

  for (Student itr: studlist){             
     std::cout << itr << std::endl;
  }
  return EXIT_SUCCESS;
}
```
@Rextester.CPP

> **Merke**: Vermeiden Sie Raw-Arrays! In der Regel lassen sich die intendierten Strukturen
> durch die Konstrukte Standardbibliothek wesentlich effizienter realisieren.

### Iteratoren
<!--
  comment: array.cpp
  ..............................................................................
  1. Realisierung über einen Interator under Verwendung von `std::begin()` und `std::end()`
  ```cpp
  #include <iterator>
  for(auto itr = std::begin(vec); itr != std::end(vec); itr++){ }
  ```
  Die Verwendung der globalen Methoden `std::begin()` und `std::end()`
  realisiert die zugehörigen Memberfunktionen der entsprechenden Containerklasse.
  2. Anwendung ohne Bereichskontext zum Beispiel bei der Suche  
  ```cpp
  #include <algorithm>
  auto zwei = std::find(vec.begin(), vec.end(), 2);
  vec.erase(zwei);
  ```
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-->

Iteratoren definieren ein allgemeineres Konzept als die zuvor gezeigten Pointer
im Umgang mit Arrays. Die darauf anwendbaren Operationen sind:

+ Dereferenzierung
+ Vergleich `itA == itB`
+ Inkrementierung `it++` oder `++it`

wobei die letzten beiden für die "Navigation" über den Daten eines container_Bs
Verwendung findet:

<table class="lia-inline lia-table">
    <thead class="lia-inline lia-table-head">
        <tr>
          <th>Iteratormethoden Modus</th>
          <th>Modus</th>
          <th>Unterstützte Container</th>
        </tr>
    </thead>
        <tr class="lia-inline lia-table-row">
          <td>
          <code><pre 	style = "Lucida Console; font-size: 14px;font-style: normal; ">vector&lt;int> itr;
itr = itr + 2;
itr = itr -4;
if (itr2 > itr1) ... ;
++itr;
--itr; </pre></code></td>
          <td>Wahlfreier Zugriff / "Random Access Iterator"</td>
          <td>vector, deque, array</td>
        </tr>
        <tr class="lia-inline lia-table-row">
          <td>
          <code><pre 	style = "Lucida Console; font-size: 14px;font-style: normal; ">list&lt;int> itr;
++itr;
--itr; </pre></code></td>
          <td>Bidirektionaler Iterator </td>
          <td>list, set/multiset, map/multimap</td>
        </tr>
        <tr class="lia-inline lia-table-row">
          <td>
          <code><pre 	style = "Lucida Console; font-size: 14px;font-style: normal; ">forward_list&lt;int> itr;
++itr;</pre></code></td>
          <td> Vorwärts Interator</td>
          <td>forward_list</td>
        </tr>
        <tr class="lia-inline lia-table-row">
          <td>
          <code><pre 	style = "Lucida Console; font-size: 14px;font-style: normal; ">int x = *itr;</pre></code></td>
          <td> Eingabe Iterator </td>
          <td></td>
        </tr>
        <tr class="lia-inline lia-table-row">
          <td>
          <code><pre 	style = "Lucida Console; font-size: 14px;font-style: normal; ">*itr = 100;</pre></code></td>
          <td> Ausgabe Iterator</td>
          <td></td>
        </tr>
</table>

Anmerkungen:

+ Jede Containerklasse integriert eine Implementierung des Iteratorkonzeptes für unbeschränkten Zugriff `iterator` und lesenden Zugriff `const_iterator`. Entsprechend bestehen neben `.begin()` und `.end()` auch `.cbegin()` und `.cend()` (seit C++11).
+ Die Pointerarithmetik kann für den wahlfreien Zugriff zum Beispiel mit `ìtr+=5` umgesetzt werden. Eine übergreifende Lösung bietet die `advance(itr, 5)`-Methode. Analog kann mit `distance(itr1, itr2)` der Abstand zwischen zwei Elementen bestimmt werden.   

Ein Haupteinsatzfeld ist die Segmentierung von Containerinhalten nach "innerhalb"
und "außerhalb" Elementen, wobei eine bestimmte Funktionalität auf die innerhalb
Elemente angewendet wird.

```cpp                      iterators.cpp
#include <iostream>
#include <vector>

int main()
{
  std::vector<int> vec {1,2,3,4,5,6,7,8,9};
  vec.erase(vec.begin()+3, vec.end()-1);
  for (std::vector<int>::iterator itr = vec.begin(); itr!=vec.end();  ++itr){
     std::cout << *itr << ", ";
  }
  return EXIT_SUCCESS;
}
```
@Rextester.CPP

### Containerkonzepte

                 {{0-1}}
************************************************

Container sind Objekte, die konzeptionell andere Objekte enthalten. Sie verwenden bestimmte grundlegende Eigenschaften der Objekte (Kopierfähigkeit usw.), hängen aber ansonsten nicht von der Art des Objekts ab, das sie enthalten.

**In der STL sind entsprechend alle Container templatisiert.** Neben den einzelnen
Containerklassen exisitiert ein Set von übergreifenden Algorithmen, die auf die
Container angewendet werden können.

```cpp
#include <vector>
#include <deque>
#include <list>
#include <map>      // verantwortlich für set und multiset
#include <set>      //                    map und multimap
#include <iterator>
#include <algorithm>
//...
std::vector<int> a;
std::vector<std::string> b;
```

**Welche Arten von Containern werden unterschieden?**

1. Sequenzcontainer (*Sequential containers*) ... legen die Daten in einer linearen Ordnung ab. Dabei erfolgt der Zugriff "der Reihe nach" oder anhand einer Indizierung.
2. Associative Container (*Associative containers*) ... verwenden einen beliebigen Datentyp für den Zugriff, der als Schlüssel bezeichnet wird. Dabei wird zwischen ungeordneten und geordneten assoziativen Containern unterschieden.
3. Container Adapter ... passen die in eins und zwei enthaltenen Containerkonstrukte an bestimmte Modelle an, in dem sie die Schnittstellen modifizieren.

************************************************

                   {{1-2}}
************************************************

**Welche Klassifikation sollte in Ihrem Kopf stattfinden?**

![STL Container](./img/04_STL/STLContainer.png)<!-- width="100%" -->
*Darstellung des Entscheidungsprozesses für die Anwendung eines STL-Containers* [^1]

[^1]: Mikael Persson  nach einem Entwurf von David Moore [Link](https://stackoverflow.com/questions/471432/in-which-scenario-do-i-use-a-particular-stl-container)

************************************************

                   {{2-3}}
************************************************

**Was sind die übergreifenden Methoden der STL Container?**

1. Prüfung auf "Leere"
```cpp
if (container.empty()) {std::cout << "Nicht's zu holen!";}
```

2. Zahl der Samples im Container
```cpp
std::cout << container.size();
```

3. Copy constructor
```cpp
vector<int> vec2(vec);
```

4. Löschoperation für alle Einträge
```cpp
container.clear();
```

5. Übertragung der Inhalte zwischen Containern gleichen Typs
```cpp
container_Bs.swap(container_A);
```

************************************************

                {{3-4}}
************************************************

**Vorsicht Falle!**

```cpp                      initPitfall.cpp
#include <vector>
#include <list>
#include <iostream>

int main()
{
  std::list<int> vec (5,2);// {5,2};
  for (auto it: vec){             
     std::cout << it << ", ";
  }
  return EXIT_SUCCESS;
}
```
@Rextester.CPP

> **Merke**:  "() != {}" :-)

************************************************

#### Sequenzcontainer

| Container      | Idea                                                               |
| -------------- | ------------------------------------------------------------------ |
| `array`        | statischer Container                                               |
| `vector`       | Wachstum des zugehörigen Speicher ausschließlich in einer Richtung |
| `deque`        | Wachstum des zugehörigen Speicher beiden Richtungen                |
| `list`         | Doppelt verlinkte Einträge                                         |
| `forward_list` | Doppelt verlinkte Einträge                                         |

| Eigenschaft              | array    | vector   | deque                 | list          | forward_list    |
| ------------------------ | -------- | -------- | --------------------- | ------------- | --------------- |
| dynamische Speichergröße |          | x        | x                     | x             | x               |
| vorwärts interieren      | x        | x        | x                     | x             | x               |
| rückwärts iterieren      | x        | x        | x                     | x             |                 |
| Speicheroverhead         | -        | ggf.     | gering                | ja            | Hälfte von List |
| effizientes Einfügen     | -        | am Ende  | am Ende und am Anfang | überall       | spezifisch      |
| Splicen des Containers   |          |          |                       | X             | X               |
| Iteratoren               | wahlfrei | wahlfrei | wahlfrei              | bidirektional | vorwärts        |
| Algorithmen              | alle     | alle     | alle                  | spezifisch    | spezifisch      |


**Beispiel `vector`**

<!--
style="width: 60%; max-width: 860px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii

         .capacity()
  <-------------------------->        
      .size()
  <--------------->        
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |0|1|2|3|4|5|6|7|8|?|?|?|?|?|
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+      
  ^               ^
  | .begin()      | .end()
```


```cpp                      vectorExample.cpp
#include <vector>
#include <iostream>
#include <algorithm>

int main()
{
  std::vector<int> vec;  // vec.size() == 0
  vec.push_back(3);
  vec.push_back(1);
  vec.push_back(7);
  //vec.pop_back();

  //std::vector<int> vec(1000001);
  //std::generate(vec.begin(), vec.end(), std::rand);

  std::cout << "Vector capacity/size: " << vec.capacity() << " / " << vec.size() << std::endl;  

  // Vector wahlfreier Zugriff   
  std::cout << vec[0] << ", " << vec.at(1) << std::endl;  
  //             |          |
  // ohne range check    wirft eine Ausnahme bei range Verletzung

  // Wahlfreies Löschen
  // vec.erase(vec.begin()+2);
  // Allgemeinere Lösung (unter Einbeziehung von Listen)
  std::vector<int>::iterator it = vec.begin();
  std::advance(it, 2);
  vec.erase(it);

  // Iteration mittels kompakter Iteratordarstellung (C++11)
  for (auto itr: vec){     
     itr +=1;
  }

  for (auto itr = vec.begin(); itr != vec.end(); itr++){
  	  *itr = *itr + *itr%2;
  }

  std::cout << vec.at(0) << ", " << vec.at(1) << " ..." << std::endl;  

  return EXIT_SUCCESS;
}
```
@Rextester.CPP

```cpp                      findExample.cpp
#include <vector>
#include <iostream>
#include <algorithm>

int main()
{
  std::vector<int> vec(100000000);
  std::generate(vec.begin(), vec.end(), []() {
	  return rand() % 100;
  });

  for (std::vector<int>::iterator itr = vec.begin(); itr != vec.end(); ++itr){       
     if (*itr == 99){
        std::cout << "Generic iterator found a 99 " << "at index " << itr - vec.begin() << std::endl;
        break;
     }
  }

  std::vector<int>::iterator it = std::find(vec.begin(), vec.end(), 99);
  if (it != vec.end()){
    std::cout << "Element Found" << std::endl;
  } else {
    std::cout << "Element Not Found" << std::endl;
  }

  return EXIT_SUCCESS;
}
```
@Rextester.CPP(true)


#### Assoziative Container

                   {{0-1}}
***********************************************************************


| Container  | Idea                                                  |
| ---------- | ----------------------------------------------------- |
| `set`      | sortierte Menge ohne Dublikate                        |
| `map`      | eindeutige Schlüssel auf einen Zielwert               |
| `multiset` | sortierte Menge von Elementen, die mehrfach vorkommen |
| `multimap` | Schlüssel referenzieren ggf. mehrere Einträge         |

Wie Sie in folgender Tabellarischem Vergleich sehen können, realisieren die  
assoziativen Container ein homogeneres Gesamtbild. Der zentrale Unterschied liegt in der Idee eines Schlüssels, der die Daten selbst repräsentiert oder aber auf den
eigentlichen Datensatz verweist.

<!--
style="width: 80%; max-width: 860px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii

          set                                    map

         +----+                            +----+---------+
         |  9 |                            | 9  | "Karl"  |
         +----+                            +----+---------+
         /    \                            /              \
        /      \                          /                \     
     +----+    +----+            +----+---------+    +----+---------+
     |  6 |    | 10 |            |  7 | "Fritz" |    | 10 | "Julius"|
     +----+    +----+            +----+---------+    +----+---------+
     /    \                      /              \
    /      \                    /                \
+----+    +----+           +----+---------+    +----+---------+
|  2 |    | 7  |           |  2 | "Agnes" |    | 6  | "Horst" |
+----+    +----+           +----+---------+    +----+---------+
```

| Eigenschaft             | set      | map      | multiset | multimap |
| ----------------------- | -------- | -------- | -------- | -------- |
| eindeutige Schlüssel    | ja       |          | ja       |          |
| Schlüssel zu Zielwerten |          | ja       |          | ja       |
| dynamischer Größe       | ja       | ja       | ja       | ja       |
| Overhad pro Element     | ja       | ja       | ja       | ja       |
| Speicherlayout          | Baum     | Baum     | Baum     | Baum     |
| Iteratoren              | wahlfrei | wahlfrei | wahlfrei | wahlfrei |
| Algorithmen             | alle     | alle     | alle     | alle     |

C++11 erweitert diese Kategorie jeweils unsortierte Darstellungen (vgl. Diagrammdarstellung für die Suche nach geeigneten Containern).

*********************************************************************

                             {{1-2}}
***********************************************************************
**Beispiel `set`**

Warum können die die Elemente nicht manipuliert werden?

```cpp                      setExample.cpp
#include <iostream>
#include <algorithm>
#include <set>

int main()
{
  std::set<int> myset;
  myset.insert(3);
  myset.insert(1);
  myset.insert(7);
  myset.insert(7);

  // Evaluate the existence of 7 before insertion
  auto value = 9;
  auto it = myset.find(value);
  if (it == myset.end()){
    std::cout << "No value 7 in current set, added this value" << std::endl;
    myset.insert(value);
  }

  auto ret = myset.insert(value);    // pair<std::set<int>>::iterator, bool>
  if (ret.second == false){
    it = ret.first;                // wir haben nun einen Zugriff auf den Iterator
  }

  myset.erase(it);

  for (auto const& itr: myset){     
    std::cout << itr << ", ";
  }

  return EXIT_SUCCESS;
}
```
@Rextester.CPP

Ermitteln Sie alle Einträge, deren Wert größer als 3 und kleiner als 7 ist. Wie würde sich das vorgehen unterscheiden, wenn wir mit einem `vector`-Container starten würden?

```cpp                      filterExample.cpp
#include <iostream>
#include <algorithm>
#include <set>

int main()
{
  std::set<int> myset {3, 5, 1, 2, 5, 6, 7, 4, 10};
  for (auto const& itr: myset){     
    std::cout << itr << ", ";
  }
  std::cout << std::endl;
  std::set<int>::iterator start = std::upper_bound(myset.begin(), myset.end(), 3);  // >
  std::set<int>::iterator end = std::lower_bound(myset.begin(), myset.end(), 7);  // <=
  std::cout << std::distance(start, end) << " Entries found!" << std::endl;
  std::cout << *start << " " << *end << std::endl;
  return EXIT_SUCCESS;
}
```
@Rextester.CPP


Anwendungsbeispiel: Führen Sie eine Datenbank der Studierenden einer Universität,
die bei korrekter Datenerfassung keine Duplikate zulässt.

```cpp                      setExample.cpp
#include <iostream>
#include <algorithm>
#include <set>

class Student{
  private:
     int matrikel;
     std::string name;
  public:
     Student(int matrikel, std::string name): matrikel(matrikel), name(name) {};
     friend std::ostream& operator<<(std::ostream& os, const Student& s);
     bool operator< (const Student &student2) const
     {
        return this->matrikel < student2.matrikel;
     }
};

std::ostream& operator<<(std::ostream& os,  const Student& s){  
   os << s.name << "," << s.matrikel;
 return os;
}

int main()
{
  std::set<Student> studlist = {Student(123, "Alexander"),
                                Student(789, "Karl"),
                                Student(345, "Julius"),
                                Student(789, "Karl")};

  for (auto itr: studlist){             
     std::cout << itr << std::endl;
  }
  return EXIT_SUCCESS;
}
```
@Rextester.CPP

***********************************************************************


#### Container-Adapter

Container-Adapter basieren auf Containern, um spezielle Formate zu definieren, die über entsprechende Schnittstellen verfügen. Ein Stack wird beispielsweise mit Hilfe eines Vektors realisiert werden. Der Stack bietet dem Benutzer nur eine eingeschränkte Schnittstelle an. Ein Adapter besitzt keine Iteratoren.

| Typ                   | Beschreibung                                                                                                                   | Methoden                                  |
| --------------------- | ------------------------------------------------------------------------------------------------------------------------------ | ----------------------------------------- |
| `std::stack`          | Ein Stack speichert seine Elemente in der Reihenfolge ihres Eintreffens und gibt sie in umgekehrter Reihenfolge wieder heraus. | `push(obj)`, `pop()`, `top()`             |
| `std::queue`          | FIFO-Speicher                                                                                                                  | `push(obj)`, `pop()`, `front()`, `back()` |
| `std::priority_queue` | FIFO-Speicher der den Elementen eine Priorität zuordnet.                                                                       | `push(obj)`, `pop()`, `front()`, `back()` |


### Algorithmen

Die Algorithmen, die die STL bereitstellt, lassen sich sowohl auf die vorgestellten Containerkonzepte, wie auch auf native C++ arrays anwenden.

+ sortieren
+ suchen
+ kopieren
+ umdrehen
+ füllen
+ ...

> **Merke**: Die Angabe eines Containerinhaltes mit zwei Iteratoren erfolgt
> immer halboffen $[begin, end)$. $end$ ist nicht in der Auswahl enthalten.

```cpp                      examplesAlgorithms.cpp
#include <iostream>
#include <algorithm>
#include <vector>

int main()
{
  std::vector<int> vec {3, 23, 4, 242, 1, 12, 2, 24};
  auto itr = std::min_element(vec.begin(), vec.end());
  std::sort(vec.begin(), itr);

  std::vector<int> vecNew(3);
  std::copy(itr, vec.end(), vecNew.begin());

  for (auto itr: vecNew){             
     std::cout << itr << std::endl;
  }

}
```
@Rextester.CPP

Daneben könnnen bei der Interation eigene Funktionen pro Element aufgerufen werden. Dabei unterscheidet man:

+ Funktionspointer
+ Lambdafunktionen
+ Functoren

```cpp                      exampleFunctionCall.cpp
#include <iostream>
#include <algorithm>
#include <vector>

bool isOdd(int i){
  return i%2;
}

void add2(int i){
  std::cout << i+2 << ", ";
}

int main()
{
  std::vector<int> vec {6, 23, 4, 242, 1, 12, 2, 23};
  std::find_if(vec.begin(), vec.end(), isOdd);
  std::for_each(vec.begin(), vec.end(), add2);
}
```
@Rextester.CPP

Die Beispiele können durch Lambda-Funktionen analog ausgedrückt werden. Dabei hat man den zusätzlichen Vorteil, dass der zusammengehörige Code nicht aufgesplittet und auf mehrere Positionen verteilt wird. Es sei denn, er wird mehrfach verwendet ...


## Aufgabe der Woche

1. Verschaffen Sie sich einen Überblick zu den Lambdafunktionen. Ermitteln Sie die Syntax und die Anwendung. Implementieren Sie ein Beispiel, dass unsere Studentencontainer nach spezifischen Merkmalskombinationen durchsucht.

2. Recherchieren Sie in Arbeiten zur Performance der verschiedenen Containerkonstrukte. Welchen Einfluss haben verschiedene Datenformate und Operationen auf die Größe des notwendigen Speichers und die Dauer der Ausführung.
