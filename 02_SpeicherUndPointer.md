<!--

author:   Sebastian Zug & Georg Jäger
email:    sebastian.zug@informatik.tu-freiberg.de & Georg.Jaeger@informatik.tu-freiberg.de
version:  0.0.1
language: de
narrator: Deutsch Female

import: https://raw.githubusercontent.com/LiaTemplates/Rextester/master/README.md

-->

# Vorlesung III - Speicher und Pointer

Eine interaktive Version des Kurses finden Sie unter [Link](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/SoftwareprojektRobotik/master/02_SpeicherUndPointer.md#1)

**Zielstellung der heutigen Veranstaltung**

+ Unterscheidung von Stack und Heap
+ Gegenüberstellung von Basis-Pointern und Smart-Pointern
+ Herausforderungen beim Speichermanagement

--------------------------------------------------------------------------------

## Komponenten des Speichers

Wie wird der Speicher von einem Programm eigentlich verwaltet?

<!--
style="width: 70%; max-width: 860px; display: block; margin-left: auto; margin-right: auto;"
-->
````ascii
        ------>  +----------------------------+
   höhere        | Commandozeilen Parameter   |    (1)
   Adresse       | und Umgebungsvariablen     |
                 +----------------------------+
                 | Stack                      |    (2)
                 |............................|
                 |             |              |
                 |             v              |
                 |                            |
                 |             ^              |
                 |             |              |
                 |............................|
                 | Heap                       |    (3)
                 +----------------------------+
                 | uninitialisierte Daten     |    (4)
                 +----------------------------+
                 | initialisierte Daten       |    (5)
  kleinere       +----------------------------+
  Adresse        | Programmcode               |    (6)
       ------>   +----------------------------+                              
````

|     | Speicherbestandteil  | engl.    | Bedeutung                              |
| --- | -------------------- | -------- | -------------------------------------- |
| 1   | Parameter            |          |                                        |
| 2   | Stack                |          | LIFO Datenstruktur                     |
| 3   | Heap                 |          |                                        |
| 4   |                      | data/bss |                                        |
| 5   | Initialisierte Daten | data     | Globale und statische Daten,           |
| 6   | Programmcode         | text     | teilbar, häufig als read only Speicher |


**Welche der Speicherelemente werden wann realisiert?**

# Untersuchung der Ausführbaren Datei

<table class="lia-inline lia-table">
    <thead class="lia-inline lia-table-head">
        <tr>
          <th>Code</th>
          <th>text</th>
          <th>data</th>
          <th>bss</th>
          <th>dec</th>
          <th>Bemerkung</th>
        </tr>
    </thead>
        <tr class="lia-inline lia-table-row">
          <td>
          <code><pre 	style = "Lucida Console; font-size: 14px;font-style: normal; "> #include &lt;iostream&gt;

 int main(void)
 {
     return EXIT_SUCCESS;
 } </pre></code></td>
          <td>1918</td>
          <td>640</td>
          <td style = "color: red;">8/4</td>
          <td>2566</td>
          <td>bss hängt von der Compiler-Konfiguation ab. Mit `-m32` entsteht eine lediglich 4Byte große, nicht initialisierte Variable </td>
        </tr>
        <tr class="lia-inline lia-table-row">
          <td>
          <code><pre 	style = "Lucida Console; font-size: 14px;font-style: normal; "> #include &lt;iostream&gt;

 int global;

 int main(void)
 {
    static int local;
    return EXIT_SUCCESS;
 } </pre></code></td>
          <td>1918</td>
          <td>640</td>
          <td style = "color: red;">16</td>
          <td>2566</td>
          <td> Rückgabewert, global, local = 3 x 4 Byte </td>
        </tr>
</table>


## Stack vs Heap

> Ich möchte gern dieses Format übernehmen. Gibt es da einen css, den ich referenzieren kann

| Parameter                   | Stack                          | Heap                           |
| --------------------------- | ------------------------------ | ------------------------------ |
| Allokation und Deallokation | Automatisch durch den Compiler | Manuel durch den Programmierer |
| Kosten                      | gering                         | höher                          |
| Flexibilität                | feste Größe                    | Anpassungen möglich                                |


### Stack Overflow

Welche Faktoren bestimmen eigentlich die Größe des verfügbaren Stacks?


Für Ubuntu 18.04 ergibt sich mit dem Befehl `ulimit` folgende Ausgabe:

ulimit -a
-t: cpu time (seconds)              unlimited
-f: file size (blocks)              unlimited
-d: data seg size (kbytes)          unlimited
-s: stack size (kbytes)             8192
-c: core file size (blocks)         0
-m: resident set size (kbytes)      unlimited
-u: processes                       61608
-n: file descriptors                1024
-l: locked-in-memory size (kbytes)  16384
-v: address space (kbytes)          unlimited
-x: file locks                      unlimited
-i: pending signals                 61608
-q: bytes in POSIX msg queues       819200
-e: max nice                        0
-r: max rt priority                 0
-N 15:                              unlimited

Die Stackgröße ist auf 8192kB beschränkt. Wenn wir


## Wiederholung (Raw-)Pointer vs. Referenzen

**Was war noch mal eine Referenz?**

C und C ++ unterstützen das Konzept des **Zeigers**, die sich von den meisten anderen Programmiersprachen unterscheiden. Andere Sprachen wie C#, C++(!), Java, Python etc. implementieren **Referenzen**.

Oberflächlich betrachtet sind sich Referenzen und Zeiger sehr ähnlich, in beiden
Fällen greifen wir indirekt auf einen Speicher zu:

+ Ein Zeiger ist eine Variable, die die Speicheradresse einer anderen Variablen enthält. Ein Zeiger muss mit dem Operator \* dereferenziert werden, um auf den Speicherort zuzugreifen. Pointer können auch ins "nichts" zeigen. C++11 definiert dafür den `nullptr`.

+ Eine Referenz ist ein Alias, das heißt ein anderer Name für eine bereits vorhandene Variable. Eine Referenz wird wie ein Zeiger implementiert, indem die Adresse eines Objekts gespeichert wird. Entsprechend kann eine Referenz auch nur für ein bestehendes Objekt angelegt werden.

Die Funktionsweise soll an folgendem Beispiel verdeutlicht werden:

```cpp                     Constructor.cpp
#include <iostream>

void DoSomeCalculationsByValue(int number){
  number = number * 2;
}

void DoSomeCalculationsByPointer(int* number){
  *number = *number * 2;
}

void DoSomeCalculationsByReference(int& number){
  number = number * 2;
}

int main()
{
  int number {1};
  DoSomeCalculationsByValue(number);
  std::cout << number << std::endl;
  int* ptr = &number;
  DoSomeCalculationsByPointer(ptr);  
  std::cout << *ptr << std::endl;
  int& ref = number;
  DoSomeCalculationsByReference(ref);
  std::cout << ref << std::endl;
}
```
@Rextester.CPP

**Bezug auf den Speicher**

```cpp                     Constructor.cpp
#include <iostream>

void ValueGeneration(int* number){
  int myNewValue {1};
  int* ptr = &myNewValue;
}

int main()
{
  int* newValue;
  std::cout << number << std::endl;
  int* ptr = &number;
  DoSomeCalculationsByPointer(ptr);  
  std::cout << *ptr << std::endl;
  int& ref = number;
  DoSomeCalculationsByReference(ref);
  std::cout << ref << std::endl;
}
```
@Rextester.CPP


**Nachteile bei der Verwendung von Raw Pointern**




## Smart Pointer
<!--
  comment: SelfDesignedUniquePointer.cpp
  ..............................................................................
  1. Kurzer Wechsel auf den Stack
  2. Neue Klasse AutomaticDeconstructor
     ```cpp
      Bla fasel
     ```
-->

[Link](http://pythontutor.com/cpp.html#code=%23include%20%3Ciostream%3E%0A%0Aclass%20myClass%0A%7B%0A%20public%3A%20%0A%20%20int%20data%5B5%5D%3B%0A%20%20myClass%28%29%7B%0A%20%20%20%20std%3A%3Acout%20%3C%3C%20%22Instance%20generated%22%20%3C%3C%20std%3A%3Aendl%3B%20%20%0A%20%20%7D%20%0A%20%20~myClass%28%29%7B%0A%20%20%20%20std%3A%3Acout%20%3C%3C%20%22Instance%20deleted%22%20%3C%3C%20std%3A%3Aendl%3B%0A%20%20%7D%0A%7D%3B%0A%0Aint%20main%28%29%20%7B%0A%20%20%7B%0A%20%20%20%20myClass*%20ptr%20%3D%20new%20myClass%28%29%3B%20%20%0A%20%20%7D%0A%20%20return%200%3B%0A%7D&curInstr=6&mode=display&origin=opt-frontend.js&py=cpp&rawInputLstJSON=%5B%5D)


## Aufgabe der Woche

1.
