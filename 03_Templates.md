<!--

author:   Sebastian Zug & Georg Jäger
email:    sebastian.zug@informatik.tu-freiberg.de & Georg.Jaeger@informatik.tu-freiberg.de
version:  0.0.1
language: de
narrator: Deutsch Female

import: https://raw.githubusercontent.com/LiaTemplates/Rextester/master/README.md

-->

# Vorlesung IV - Templates

Eine interaktive Version des Kurses finden Sie unter [Link](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/SoftwareprojektRobotik/master/03_Templates.md#1)

**Zielstellung der heutigen Veranstaltung**

+ Einführung in die Konzepte der generischen Programmierung unter C++
+ Abgrenzung zu den unter C# bekannten Ansätzen (*Templates* vs. *Generics*)

--------------------------------------------------------------------------------
## Kurze Erinnerung
<!--
  comment: GenericsIComparable.cs
  ..............................................................................
  Das generische Interface `IComparable<T>` erzwingt die Implementierung einer
  Methode `public int CompareTo(T)`. Dafür wird im Beispiel eine String Operation
  verwendet. Die Methode String.Compare hat folgende Spezifikation bezüglich des Rückgabewertes:
     Kleiner als 0 (null) 	strA steht in der Sortierreihenfolge vor strB.
     Zero 	strA tritt in der Sortierreihenfolge an der gleichen Position wie strB auf.
     Größer als 0 (null) 	strA steht in der Sortierreihenfolge hinter strB.
  ```csharp
  public class Student : IComparable<Student>
  {
    public string name;
    // ... and some other information
    public Student(string name){
      this.name = name;
    }
    public int CompareTo(Student that)
    {
      if (String.Compare(this.name, that.name) <= 0 ) return -1;
      else return 1;
    }
  }
  ```
-->

Innerhalb des .NET Frameworks definieren Generics Typparameter, wodurch Sie Klassen und Methoden entwerfen können, die die Angabe eines oder mehrerer Typen verzögern können, bis die Klasse oder Methode vom Clientcode deklariert und instanziiert wird. Ein Platzhalter, der, generischen Typparameter der häufig mit „T“ bezeichnet wird, definiert die Art einer Variablen, die von anderem Clientcode verwendet werden kann, ohne dass die Kosten und Risiken von Umwandlungen zur Laufzeit oder Boxingvorgängen anfallen.

Das folgende Beispiel zeigt eine Anwendung in Kombination mit einer Einschränkung
des Types, die Sicherstellt, dass in jedem Fall die angeforderte Vergleichsoperation besteht.

```csharp      GenericsIComparable.cs
using System;

namespace Rextester
{
  public class Student
  {
    public string name;
    // ... and some other information

    public Student(string name){
      this.name = name;
    }
  }

  public class Program{

    static void SwapIfGreater<T>(ref T lhs, ref T rhs)
                  where T : System.IComparable<T>
    {
        T temp;
        if (lhs.CompareTo(rhs) > 0)
        {
            temp = lhs;
            lhs = rhs;
            rhs = temp;
        }
    }

    public static void Main(string[] args)
    {
        int a = 5;
        int b = 7;
        SwapIfGreater<int>(ref a, ref b);
        System.Console.WriteLine("a=" + a + ", b=" + b);
    }
  }
}
```
@Rextester.eval(@CSharp)


## Arten von Templates

Templates (englisch für Schablonen oder Vorlagen) sind ein Mittel zur Typparametrierung in C++. Templates ermöglichen generische Programmierung und **typsichere** Container.
In der C++-Standardbibliothek werden Templates zur Bereitstellung typsicherer Container, wie z. B. Listen, und zur Implementierung von generischen Algorithmen, wie z.B. Sortierverfahren, verwendet.

### Funktionstemplates
<!--
  comment: FunctionOverloading.cs
  ..............................................................................
  Einführen einer neuen Methode `print(double value)` und Delegation von `print(int value)` über einen expliziten Cast. Ergänzung einer Methode für `const char[]`
  ```cpp
  void print(const char* value){
    std::cout << value;
  }
  void print (double value){
    std::cout << value << std::endl;
  }
  void print (int value){
    print ((double)value);
  }
  ```
-->
Welche Möglichkeiten haben wir unter C++ schon kennengelernt, die einenn variablen
Umgange von identischen Funktionsaufrufen mit unterschiedlichen Typen realsieren?
Dabei unterstützen uns Cast-Operatoren und das Überladen von Funktionen.

```cpp                     FunctionOverloading.cpp
#include <iostream>

void print (int value){
  std::cout << value << std::endl;
}

int main()
{
  print(5);
  print(10.234);
  //print("TU Freiberg");
  return EXIT_SUCCESS;
}
```
@Rextester.CPP

Ein Funktions-Template (auch fälschlich Template-Funktion genannt) verhält sich wie eine Funktion, die Argumente verschiedener Typen akzeptiert oder unterschiedliche Rückgabetypen liefert. Die C++-Standardbibliothek enthält eine Vielzahl von Funktionstemplates, die folgendem Muster einer selbstdefinierten Funktion entsprechen.

```cpp                     FunctionTemplate.cpp
#include <iostream>

template<typename T>          // Definition des Typalias T
void print (T value){         // Innerhalb von print wirkt T als Platzhalter
  std::cout << value << std::endl;
}

int main()
{
  print(5);
  print(10.234);
  print("TU Freiberg");
  return EXIT_SUCCESS;
}
```
@Rextester.CPP

Was macht der Compiler daraus? Lassen Sie uns prüfen, welche Symbole erzeugt wurden. Offenbar wurde aus unserem Funktionstemplate entsprechend der Referenzierung 3 unterschiedliche Varianten generiert. Mit `c++filt` kann der Klarname rekonstruiert werden.

```bash
>g++ Templates.cpp
>nm a.out | grep print
0000000000000a26 W _Z5printIdEvT_
00000000000009f2 W _Z5printIiEvT_
0000000000000a64 W _Z5printIPKcEvT
>c++filt _Z5printIdEvT_
void print<double>(double)
```

Damit sollten folgende Begriffe festgehalten werden:

+ ein Funktionstemplate definiert ein Muster oder eine Schablone für die eigentliche Funktionalität
+ das Funktionstemplate wird initialisiert, in dem die Templateparameter festgelegt werden.
+ der Compiler übernimmt das Matching und überprüft, ob eine Zuordnung des spezifischen Datentypes möglich ist und generiert eine Templatefunktion.

Warum funktioniert das Konzept bisher so gut? Alle bisher verwendeten Typen bedienen den Operator `<<` für die Stream-Ausgabe. Eine Datenstruktur, die dieses Kriterium nicht erfüllt würde einen Compilerfehler generieren.

Zwei Fragen bleiben noch offen:

| Frage                                                                            | Antwort |
| -------------------------------------------------------------------------------- | ------- |
| Muss der Templateparameter zwingend angegeben werden?                            | Nein, wenn Sie im nachfolgenden Codebeispiel die Funktion `print(int value)` entfernen, funktioniert die Codegenerierung noch immer. Der Compiler erkennt den Typen anhand des übergebenen Wertes.         |
| Ist ein Nebeneinander von Funktionstemplates und allgemeinen Funktionen möglich? | Ja, zunächst wird geprüft, ob eine nicht-templatisierte Funktion vorliegt und erst dann die Realisierung der templatisierten Funktion erzeugt. Untersuchen Sie das Beispiel mit `nm` und `c++filter`!      |

```cpp                     FunctionTemplate.cpp
#include <iostream>

void print (int value){
  std::cout << "Die Funktion wird aufgerufen" << std::endl;
  std::cout << value << std::endl;
}

template<typename T>
void print (T value){
  std::cout << "Die Templatefunktion wird aufgerufen" << std::endl;
  std::cout << value << std::endl;  
}

int main()
{
  print(5);
  print<int>(5);
  return EXIT_SUCCESS;
}
```
@Rextester.CPP

Darüber hinaus sind Templatespezialisierungen möglich, die Ausnahmen von der generellen Abbildungsregel ermöglichen.

```cpp                     TemplateSpecialization.cpp
#include <iostream>

template<typename T>
void print (T value){
  std::cout << value << std::endl;  
}

template <> void print<bool> (bool value){
  std::cout << (value ? "true" : "false") << std::endl;
}

int main()
{
  print(5);
  print<bool>(true);
  return EXIT_SUCCESS;
}
```
@Rextester.CPP

Damit lassen sich dann insbesondere für Templates mit mehr als einem Typparameter komplexe Regelsets aufstellen:

```cpp
template<class T, class U>  // Generische Funktion
void f(T a, U b) {}

template<class T>           // Überladenes Funktions-Template
void f(T a, int b) {}

template<>                  // Vollständig spezialisiert; immer noch Template
void f(int a, int b) {}
```

Daraus ergibt sich folgende Aufrufstruktur:

| Aufruf                | Adressierte Funktion                          |
| --------------------- | --------------------------------------------- |
| `f<int, int> (3,7);`  | Generische Funktion                           |
| `f<double> (3.5, 5);` | Überladenes Funktionstemplate                 |
| `f(3.5, 5);`          | Überladenes Funktionstemplate                 |
| `f(3, 5);`            | Vollständig spezialisiertes Funktionstemplate |

Methoden-Templates sind auch nur Funktionstemplates, dass heißt die gereade vorstellten Mechanismen lassen sich im Kontext Funktion, die einer Klasse zugeordnet ist analog umsetzen.

### Klassentemplates
<!--
  comment: ClassTemplate.cpp
  ..............................................................................
  1. Wie können wir unser Konstrukt auf dem Heap ablegen?
  ```cpp
  #include <memory>
  std::shared_ptr<OptionalVariable<double>> Para3 = std::make_shared<OptionalVariable<double>>(242.23424);
  std::cout << Para3->getVariable();
  ```
  2. Im Weiteren lassen sich auch für Member der Klasse Spezialisierungen definieren
  ```cpp
  template <>
  void OptionalVariable<std::string>::clear(){
     std::cout << "Lösche den String" << std::endl;
     variable = "-";
  }
  ```
-->

    {{0-1}}
*******************************************************************************

**Grundsätzliche Idee**

Ein Klassen-Template geht einen Schritt weiter und wendet die Template-Konzepte einheitlich auf eine Klasse an. Dies können selbstgeschriebene
Klassen sein oder Klassen, die zum Beispiel in der C++ Standardbibliothek
(STL) eingebettet sind und Container für Daten definieren.

Das folgende Bespiel definiert ein eigenes Klassentemplate, dass die Verwaltung einer Variablen übernimmt. Sofern ein gültiger Wert hinterlegt wurde, ist die entsprechende
Kontrollvariable gesetzt:

```cpp                    ClassTemplate.cpp
#include <iostream>
#include <list>
#include <string>

template <typename T>
class OptionalVariable{
     T variable;
     bool valid;
   public:
     OptionalVariable() : valid(false) {}
     OptionalVariable(T variable): variable(variable), valid(true) {}
     T getVariable(){
       if (!valid){
          std::cerr << "Variable not valid!" << std::endl;
       }
       return variable;
     }
     void clear(){
         valid = false;  
     }
};

int main(){
    OptionalVariable<int> Para1 = OptionalVariable<int>(5);
    std::cout << Para1.getVariable();

    OptionalVariable<std::string> Para2 = OptionalVariable<std::string> ("Das ist ein Test");
    Para2.clear();
    std::cout << Para2.getVariable();
}
```
@Rextester.CPP

Auch in diesem Kontext kann eine Spezialisierung von Templates für bestimmte
Typen erfolgen, integrieren Sie beispielsweise ein seperates Template die Methode `clear` für `std::string`, in der die zugehörige Variable auf einen leeren Ausdruck `""` gesetzt wird.  

Nur am Rande ... Was wären alternative C++ Umsetzungen für die genannte Anwendung:

+ Verwendung eines templatisierten SmartPointers ohe das eigene Template
+ Addressierung der Variablen mittels Zeiger und dessen Setzen auf `nullptr` für den Fall einess ungültigen Eintrages

*******************************************************************************

     {{1-2}}
*******************************************************************************

**Klassentemplates der STL**

Anwendungsseitig spielen Templates im Zusammenhang mit den Containern der STL
eine große Bedeutung. Die Datenstrukturen sind (analog zu C#) als Klassentemplates
realisiert.

```cpp
#include <iostream>
#include <list>
#include <string>

class MyClass{
   private:
     std::string name;
   public:
     MyClass(std::string name) : name(name) {}
};

int main(){
    // Beispiel 1
    std::list< int > array = { 3, 5, 7, 11 };
    for(auto i = std::begin(array);
        i != std::end(array);
        ++i
    ){
        std::cout << *i << ", ";
    }

    // Beispiel 2
    std::list<MyClass> objects;

    objects.push_back()
    for (it = objects.begin(); it!=objects.end(); it++) {
      delete *it;
    }
}
```
@Rextester.CPP


*******************************************************************************

     {{2-3}}
*******************************************************************************

**Mehrfache Template-Parameter**

Darüber hinaus können (wie auch bei den Funktionstemplates) mehrere Datentypen
in Klassentemplates angegeben werden. Der vorliegende Code illustriert dies
am Beispiel einer Klassifikation, die zum Beispiel mit einem Neuronalen Netz
erfolgte. Ein Set von Features wird auf eine Klasse abgebildet.

```cpp                           Container.cpp
#include <iostream>
#include <list>
#include <string>
#include <iterator>

template<typename T, typename U>
class Classification{
   private:
     std::list<T> input;
     U category;
   public:
     template<typename ContainerType>
     Classification(ContainerType t, U u) : input(t.begin(), t.end()), category(u) {}
     void print(std::ostream& os) const{
        os << "Category: " << this->category << std::endl;
        for(const auto& v : this->input)
        {
            os << v << std::endl;
        }
     }
};

int main()
{
    std::list<int> samples {23, 2, 19, -12};
    std::string category  {"Person"};
    Classification<int, std::string> datasample {samples, category};
    datasample.print(std::cout);
}
```
@Rextester.CPP

Ausgehend davon folgt dann sofort die Frage, ob sich die Zuordnung von mehr als
einem formalen Datentypen auch bei der Spezialisierung berücksichtigen lässt.
Ja, in vollem Umfang lassen sich die Kombinationen der Typparameter partiell UND
vollständig spezialisieren.

> Merke: Funktionstemplates können nicht partiell spezialisiert werden!


*******************************************************************************

     {{3-4}}
*******************************************************************************


**Templates und Vererbung**

1. Vererbung von einem Klassentemplate auf ein Klassentemplate

Zwischen Klassentemplates können Vererbungsrelationen bestehen, wie zwischen konkreten Klassen. Dabei sind verschiedene Konfigurationen möglich:

+ die erbende Klasse nutzt den gleichen formalen Datentypen wie die Basisklasse (vgl. nachfolgendes Beispiel 1)
+ die erbende Klasse erweitert das Set der Templates um zusätzliche Parameter
+ die erbende Klasse konkretisiert einen oder alle Parameter
+ die Vererbungsrelation besteht zu einem formalen Datentyp, der dann aber eine Klasse sein muss. (Beispiel 2)

```cpp
#include <iostream>
#include <list>
#include <string>

template<typename T>
class Base{
   private:
     T data;
   public:
     void set (const T& value){
        data = value;
     }
};

template<typename U>
class Derived: public Base<U>{
  public:
    void set (const U& value){
      Base<U>::set(value);
      // some additional operations happen here
    }
};

int main(){
    Derived<int> A;
    A.set(5);
}
```
@Rextester.CPP

Im Beispiel 2 erbt die abgeleitete Klasse unmittelbar vom Templateparameter.

```cpp                 InheritanceFromFormelType.cpp
#include <iostream>
#include <list>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream> // stringstream

class Data{
  private:
    double value;
  public:
    Data(double value): value(value) {}
    void set(double value) {
      value = value;
    }
    double get(){
      return value;
    }
};

template<typename T>
class TimeStampedData: public T{
  private:
    std::time_t timestamp;
  public:
    TimeStampedData(double value): T(value) {
      auto time_point = std::chrono::system_clock::now();
      timestamp = std::chrono::system_clock::to_time_t(time_point);
    }
    void print(std::ostream& os) const{
      std::stringstream ss;
      os << std::put_time(std::localtime(&timestamp), "%Y-%m-%d %X");
      os << " Value " << this.get();
    }
};

int main(){
    TimeStampedData<Data> A {5};
    A.print(std::cout);
}
```
@Rextester.CPP

Was ist kritisch an dieser Implementierung?

+ Die formelle Festlegung auf `double` in der Klasse `Data` schränkt die Wiederverwendbarkeit drastisch ein!
+ Das Klassentemplate setzt ein entsprechendes Interface voraus, dass einen Konstruktor, eine set-Funktion und ein Member data vom Typ double erwartet.

Wir müssen also prüfen, ob die Member des Templateparameters mit diesen Signaturen übereinstimmen.

```
#include <type_traits>

template<typename T>
class YourClass {

    YourClass() {
        // Compile-time check
        static_assert(std::is_base_of<BaseClass, T>::value, "type parameter of this class must derive from BaseClass");

        // ...
    }
}
```

Aktuell besteht keine Unterstützung von *constrains* analog zum Generics-Konzepts unter C#. Hier werden in dem akutellen C++20 Standards mit *concepts* neue Möglichkeiten definiert. Prüfen Sie ggf. ob Ihr Compiler diese unterstützt!

Zum Beispiel für den g++ unter ... https://gcc.gnu.org/projects/cxx-status.html

*******************************************************************************

## Template Parameter

     {{0-1}}
*******************************************************************************

Bei der Definition eines Templates kann entweder `class` oder aber `typename`
verwendet werden. Beide Formate sind in den meisten Fällen austauschbar.

```cpp
template<class T>
class Foo
{
};

template<typename T>
class Foo
{
};
```

Allerdings gibt es Unterschiede bei der Verwendung in Bezug auf Template-Templates (bis C++17) und bei abhängigen Typen.

>Beispiel !

*******************************************************************************

     {{1-2}}
*******************************************************************************

**1. Datentypen**

... waren Gegenstand des vorangegangenen Kapitels.

**2. Nichttyp-Parameter**

Nichttyp-Parameter sind Konstanten, mit denen Größen, Verfahren oder Prädikate als Template-Parameter übergeben werden können. Als Nichttyp-Template-Parameter sind erlaubt:

+ ganzzahlige Konstanten (inklusive Zeichenkonstanten),
+ Zeigerkonstanten (Daten- und Funktionszeiger, inklusive Zeiger auf Member-Variablen und -Funktionen) und
+ Zeichenkettenkonstanten

Verwendung finden Nichttyp-Parametern z. B. als Größenangabe bei std::array oder als Sortier- und Suchkriterium bei vielen Algorithmen der Standardbibliothek.

```cpp                    ConstantsAsTemplateParameters.cpp
#include <iostream>
#include <cstddef>          // Für den Typ size_t

template <std::size_t N, typename T>
void array_init(T (&array)[N], T const &startwert){
    for(std::size_t i=0; i<N; ++i)
        array[i]=startwert + i;
}

int main(){
    const std::size_t size {10};
    int A[size];
    array_init<size, int>(A, 6);
    for (unsigned int i=0; i < size; i++){
    	std::cout << A[i] << " ";
    }
    std::cout << std::endl;
    return EXIT_SUCCESS;
}
```
@Rextester.CPP

**Template-Templates**

Als Template-Parameter können aber auch wiederum Templates genutzt werden.

```cpp
template <template <typename, typename> class Container,
          typename Type>
class Example {
    Container<Type, std::allocator <Type> > myContainer;
    //...
};

// Anwendung
Example <std::deque, int> example;
```

Anwendung findet dies zum Beispiel in der Implementierung der `std::stack`
Klasse, die ohne weitere Parameter eine `std::deque` als Datenstruktur
verwendet. Diese wird in der Klassendeklaration als Standardparameter übergeben.
Der zugrunde liegende Container kann eine der Standard-Container-Klassenvorlagen
sein, der aber folgenden Vorgänge unterstützen muss:

+ empty
+ size
+ back
+ push_back
+ pop_back

```
template<
    class T,
    class Container = std::deque<T>    // <- Warum wird hier auf die erneute      
>                                      // Templatisierung verzichtet?
class stack;
```

Für andere Container gibt es ähnliche Realisierungen. Der Beitrag von Herb Sutter gibt dazu eine fundierte Diskussion der Performanceunterschiede von `std::list`, `std::vector` und `std::deque` [Link](http://www.gotw.ca/gotw/054.htm).

**Parameter-Packs**

Der Vollständigkeit halber soll noch darauf hingewiesen werden, dass auch eine
variable Zahl von Templateparametern realisiert werden kann. Diese Flexibilität
wird dann durch `...` als Platzhalter ausgedrückt.


*******************************************************************************


## Vergleich generischer Konzepte unter C# und C++

https://www.artima.com/intv/generics.html#part2

| Kriterium                          | C# Generics  | C++ Templates          |
| ---------------------------------- | ------------ | ---------------------- |
| Template-Ziele                     | Klassen      | Klassen und Funktionen |
| Typisierung                        | Stark        | Schwach                |
| Instanziierung                     | zur Laufzeit | zur Conpilezeit        |
| Default-Werte                      | nein         | ja                     |
| Vollständige Spezialisierung       | nein         | ja                     |
| Partielle Spezialisierung          | nein         | ja                     |
| Nicht-Typen als Template-Parameter | nein         | ja                     |
| Typ-Parameter als Basisklassen     | nein         | ja                     |
| Template-Templates                 | nein         | ja                     |

Und wie geht es weiter unter C++? Die Template-Metaprogrammierung greift noch weiter und definiert eine eigene Spracheebene in C++. Eine sehr anschauliche erste Einführung dazu bietet der Blogbeitrag von *-AB-* auf `coding::board` unter folgendem [Link](https://www.coding-board.de/resources/c-template-metaprogramming-101.48/)

## Aufgabe der Woche

1. Recherchieren Sie die Möglichkeit *default* Werte bei der Angabe der Templateparameter zu berücksichtigen.

2. Implementieren Sie ein Beispiel, dass partielle und vollständige Spezialisierung einer Templateklasse realisiert.
