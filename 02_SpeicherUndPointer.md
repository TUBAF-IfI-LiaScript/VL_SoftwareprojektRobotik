<!--

author:   Sebastian Zug & Georg Jäger
email:    sebastian.zug@informatik.tu-freiberg.de & Georg.Jaeger@informatik.tu-freiberg.de
version:  0.0.1
language: de
narrator: Deutsch Female

import:   https://github.com/liascript/CodeRunner

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/VL_SoftwareprojektRobotik/master/02_SpeicherUndPointer.md#1)

# Speicher und Pointer

| Parameter            | Kursinformationen                                                                                                                                                                             |
| -------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Veranstaltung:**   | `Softwareprojekt Robotik`                                                                                                                                                                     |
| **Semester**         | `Wintersemester 2021/22`                                                                                                                                                                      |
| **Hochschule:**      | `Technische Universität Freiberg`                                                                                                                                                             |
| **Inhalte:**         | `Grundlagen der Speicherverwaltung unter C++`                                                                                                                                                |
| **Link auf GitHub:** | [https://github.com/TUBAF-IfI-LiaScript/VL_Softwareentwicklung/blob/master/02_SpeicherUndPointer.md](https://github.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/blob/master/02_SpeicherUndPointer.md) |
| **Autoren**          | @author                                                                                                                                                                                       |

![](https://media.giphy.com/media/EizPK3InQbrNK/giphy.gif)

--------------------------------------------------------------------------------

**Zielstellung der heutigen Veranstaltung**

+ Unterscheidung von Stack und Heap
+ Gegenüberstellung von Basis-Pointern und Smart-Pointern
+ Herausforderungen beim Speichermanagement

> **Organisatorisches** Zur Terminfindung für die erste Blockveranstaltung haben wir eine Doodle-Umfrage aufgestzt, ihr findet sie [hier](https://doodle.com/poll/29reuubiuqzyvfh4?utm_source=poll&utm_medium=link). Tragt euch dort bitte bis zum 09.11.21 ein damit wir einen Termin festlegen können.

--------------------------------------------------------------------------------

## Fragen aus der vergangenen Woche

Wie war das noch mal `Referenzen` und `Pointer`?

[PythonTutor](https://pythontutor.com/cpp.html#code=%23include%20%3Cstdio.h%3E%0A%0Avoid%20function%28int%20*aux%29%7B%0A%20%20*aux%2B%2B%3B%0A%20%20printf%28%22Call%20by%20'Reference'%3A%20%25i%5Cn%22,%20*aux%29%3B%0A%7D%0A%0Avoid%20function%28int%20aux%29%7B%0A%20%20aux%2B%2B%3B%0A%20%20printf%28%22Call%20by%20Value%3A%20%25i%5Cn%22,%20aux%29%3B%0A%7D%0A%0Aint%20main%28%29%20%7B%0A%20%20int%20i%20%3D%203%3B%20%0A%20%20int%20j%20%3D%205%3B%0A%20%20%0A%20%20//%20Initialisierung%0A%20%20//%20--------------------------------------%0A%20%20//%20Pointer%20auf%20eine%20Variable%0A%20%20int%20*ptr%20%3D%20%26i%3B%20%0A%20%20int%20*empty_prt%3B%0A%20%20%0A%20%20//%20Referenz%20auf%20eine%20Variable%0A%20%20int%20%26ref%20%3D%20i%3B%20%0A%20%20int%20%26ref_b%20%3D%20i%3B%20%0A%20%20//int%20%26empty_ref%3B%20//%20Error!%0A%20%20%0A%20%20//%20Reassignment%0A%20%20ptr%20%3D%20%26j%3B%0A%20%20//ref%20%3D%20i%3B%20//%20Reassigment%20nicht%20m%C3%B6glich%20%0A%20%20%0A%20%20//%20Indirection%20-%20Funktioniert%20nicht%20mit%20Referenzen%0A%20%20int%20**ptr_ptr%3B%20%20//it%20is%20valid.%0A%20%20ptr%20%3D%20%26i%3B%0A%20%20ptr_ptr%20%3D%20%26ptr%3B%0A%20%20printf%28%220x%25p%20points%20at%20%25i%5Cn%22,%20%28void%20*%29ptr,%20i%29%3B%0A%20%20printf%28%220x%25p%20points%20at%200x%25p%5Cn%5Cn%22,%20%28void%20*%29ptr_ptr,%20*ptr_ptr%29%3B%0A%20%20%20%20%0A%20%20//%20Funktionsaufruf%0A%20%20function%28ptr%29%3B%0A%20%20printf%28%22Resultierendes%20i%3A%20%25i%5Cn%22,%20j%29%3B%0A%20%20function%28ref%29%3B%0A%20%20printf%28%22Resultierendes%20i%3A%20%25i%5Cn%22,%20i%29%3B%0A%20%20return%200%3B%0A%7D&mode=edit&origin=opt-frontend.js&py=cpp_g%2B%2B9.3.0&rawInputLstJSON=%5B%5D)

> *Achtung:* Im Code ist ein Pointer Anfängerfehler verborgen. Dessen Wirkung sollte mit der Ausgabe in Zeile 41 deutlich werden - Finden Sie ihn?


## Komponenten des Speichers

Wie wird der Speicher von einem C++ Programm eigentlich verwaltet? Wie wird diese Struktur ausgehend vom Start eines Programmes aufgebaut?

<!--
style="width: 70%; max-width: 660px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii
        ------>  +----------------------------+
   höhere        | Kommandozeilen Parameter   |    (1)
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
```

|     | Speicherbestandteil    | engl. | Bedeutung                              |
| --- | ---------------------- | ----- | -------------------------------------- |
| 1   | Parameter              |       |                                        |
| 2   | Stack                  |       | LIFO Datenstruktur                     |
| 3   | Heap                   |       | "Haldenspeicher"                       |
| 4   | Uninitialisierte Daten | .bss  |                                        |
| 5   | Initialisierte Daten   | .data | Globale und statische Daten,           |
| 6   | Programmcode           | .text | teilbar, häufig als read only Speicher |

### Untersuchung der ausführbaren Datei

**Welche Anteile der Speicherstruktur sind zur Compilezeit analysierbar?**

```cpp     memory.cpp
#include <iostream>

int main(void)
{
    return EXIT_SUCCESS;
}
```

```bash
gcc memory.c -o memory
size memory
text       data        bss        dec        hex    filename
1918       640         8          2566       a06    memory
```

Der reine Quellcode wird im `.text` Segment abgelegt. Im `.data` und `.bss` Segment werden lediglich die globalen und statischen, lokalen
Variablen abgelegt.

> Achtung: Im Beispiel wurde keine Optimierung verwendet. Je nach Konfiguration ergeben sich hier unterschiedliche Resultate!

<lia-keep>
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
        <tr class="lia-inline lia-table-row">
          <td>
          <code><pre 	style = "Lucida Console; font-size: 14px;font-style: normal; "> #include &lt;iostream&gt;

 int global = 5;

 int main(void)
 {
    static int local = 3;
    return EXIT_SUCCESS;
 } </pre></code></td>
          <td>1918</td>
          <td style = "color: red;">648</td>
          <td >8</td>
          <td>2574</td>
          <td> Initialisierte globale/statische Variablen </td>
        </tr>
        <tr class="lia-inline lia-table-row">
          <td>
          <code><pre 	style = "Lucida Console; font-size: 14px;font-style: normal; "> #include &lt;iostream&gt;

 int main(void)
 {
    int local = 3;
    return EXIT_SUCCESS;
 } </pre></code></td>
          <td>1918</td>
          <td >640</td>
          <td >8</td>
          <td>2566</td>
          <td> Keine globalen/statischen Variablen </td>
        </tr>
</table>
</lia-keep>


### Stack vs Heap
<!--
  comment: StackvsHeap.cpp
  ..............................................................................
  1. Illustriere die Bedeutung und die Unterscheidung von Heap und Stack! Verweis auf die Warnings
  ```cpp
  int* CreateArray(){
     int array[] = {1,2,3,4,5};
     return array;
   }
  int main()
  {
     int* a = CreateArray();
     std::cout << a [2] << std::endl;
  ...
  ```
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-->

**Zunächst mal ganz praktisch, was passiert auf dem Stack?**

Ausgangspunkt unserer Untersuchung ist ein kleines Programm, das mit dem gnu Debugger `gdb` analysiert wurde:

```
g++ -m32 StackExample.cpp -o stackExample
gdb stackExample
(gdb) disas main
(gdb) disas calc
```

```cpp                     StackExample.cpp
#include <iostream>

int calc(int factor1, int factor2){
  return factor1 * factor2;
}

int main()
{
  int num1 {0x11};
  int num2 {0x22};
  int result {0};
  result = calc(num1, num2);
  std::cout << result << std::endl;
  return EXIT_SUCCESS;
}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

Was passiert beim starten des Programmes und beim  Aufruf der Funktion `calc` "unter der Haube"? Schauen wir zunächst die Einrichtung des Stacks von Seiten der `main` funktion bis zur Zeile 12.

```asm
0x06ed <+10>:    push   %ebp
0x06ee <+11>:    mov    %esp,%ebp
0x06f0 <+13>:    push   %ebx
0x06f1 <+14>:    push   %ecx
0x06f2 <+15>:    sub    $0x10,%esp
... # Initialisierung der Variablen
0x0700 <+29>:    movl   $0x11,-0x14(%ebp)
0x0707 <+36>:    movl   $0x22,-0x10(%ebp)
0x070e <+43>:    movl   $0x0,-0xc(%ebp)
... # Aufruf des Unterprogramms
0x071b <+56>:    call   0x6cd <calc(int, int)>
0x0720 <+61>:    add    $0x8,%esp
0x0723 <+64>:    mov    %eax,-0xc(%ebp)
0x0726 <+67>:    sub    $0x8,%esp
... # Aufruf Betriebssystemschnittstell für Ausgabe
0x0733 <+80>:    call   0x570 <_ZNSolsEi@plt>
....
```

<!--
style="width: 100%; max-width: 660px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii
                                    kleinere Adresse

            +----------------> |                           |
            |                  +---------------------------+
            |          rbp-x1c | ????                      | ^
            |                  +---------------------------+ |
            |          rbp-x14 | num1                      | |  4 x 4Byte
      +------------+           +---------------------------+ |  = 0x10
rsb   |stackpointer|   rbp-x10 | num2                      | |  = 16d Byte
      +------------+           +---------------------------+ |
                       rbp-x0c | result                    | v
                               +---------------------------+
                       rbp-x08 | ecx                       |  Gesicherte
                               +---------------------------+  Register
                       rbp-x04 | ebx                       |
      +------------+           +---------------------------+
rbp   |basepointer |------->   | Alter Base/Framepointer   |  neuer Frame
      +------------+           +---------------------------+  ..........
                               | Alter Instruktionspointer |
                               | ...                       |

                                     größere Adresse                           .
```

Nun rufen wir die Funktion `calc` auf und führen die Berechnung aus. Dafür
wird ein neuer Stackframe angelegt. Wie entwickelt sich der Stack ausgehend von
dem zugehörigen Assemblercode weiter?

```
0x06cd <+0>:  push   %ebp
0x06ce <+1>:  mov    %esp,%ebp
0x06da <+13>: mov    0x8(%ebp),%eax
0x06dd <+16>: imul   0xc(%ebp),%eax
0x06e1 <+20>: pop    %ebp
0x06e2 <+21>: ret
```

<!--
style="width: 100%; max-width: 660px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii
                                    kleinere Adresse

            +----------------> | ...                       |
            |                  +---------------------------+
            |           +----> |   Alter Framepointer   ...|.......
            |           |      +---------------------------+      .
            |           |      | Alter Instruktionspointer |      .
            |           |      +---------------------------+      .
            |           |      | ????                      |      .
            |           |      +---------------------------+      .
            |           |      | num1                      |      .
      +------------+    |      +---------------------------+      .
rsb   |stackpointer|    |      | num2                      |      .
      +------------+    |      +---------------------------+      .
                        |      | result                    |      .
                        |      +---------------------------+      .
                        |      | ecx                       |      .
                        |      +---------------------------+      .
                        |      | ebx                       |      .
      +------------+    |      +---------------------------+      .
rbp   |basepointer |----+      | Alter Framepointer        | <.....
      +------------+           +---------------------------+
                               | Alter Instruktionspointer |
                               +---------------------------+
                               | ...                       |

                                     größere Adresse                           .
```

**Heap**

Der Heap ist ein dedizierter Teil des RAM, in dem von der Applikation Speicher dynamisch belegt werden kann. Speichergrößen werden explizit angefordert und wieder frei gegeben.

Unter C(!) erfolgt dies mit den Funktionen der Standardbibliothek. C++ übernimmt diese Funktionalität.

```c
#include <stdlib.h>
void *malloc(size_t size);
void *calloc(size_t n, size_t size);
void *realloc(void *ptr, size_t size)
free(void *ptr);
```

C++ erhöht den Komfort für den Entwickler und implementiert ein alternatives Konzept.

```cpp
new «Datentyp» »(«Konstruktorargumente»)«
delete «Speicheradresse»
```

|                   | `new`                        | `malloc`                        |
| ----------------- | ---------------------------- | ------------------------------- |
| Bedeutung         | Schlüsselwort, Operator      | Funktion der Standardbibliothek |
| Rückgabewert      | Pointer vom Typ des Objektes | `void` Pointer                  |
| Fehlerfall        | Ausnahme                     | NULL Pointer                    |
| Größe             | wird vom Compiler bestimmt   | muss manuell festgelegt werden  |
| Überschreibarkeit | ja                           |                                 |


**Beispiel**

```
#include <iostream>

struct Bruch{
  int zaehler = 1;
  int nenner = 1;

  Bruch(int z, int n) : zaehler{z}, nenner{n} {};
  void print(){
    std::cout << zaehler << "/" << nenner << std::endl;
  }
};

int main(){
  Bruch einhalb {1,2};
  Bruch* einviertel = static_cast<Bruch*>(malloc(sizeof(Bruch)));
  einviertel->zaehler = 1;
  einviertel->nenner = 4;
  einviertel->print();
  Bruch* einachtel = new Bruch(1,8);
  einachtel->print();
  return EXIT_SUCCESS;
}
```

[Link auf pythontutor](http://pythontutor.com/cpp.html#code=%23include%20%3Ciostream%3E%0A%0Astruct%20Bruch%7B%0A%20%20int%20zaehler%20%3D%201%3B%0A%20%20int%20nenner%20%3D%201%3B%0A%0A%20%20Bruch%28int%20z,%20int%20n%29%20%3A%20zaehler%7Bz%7D,%20nenner%7Bn%7D%20%7B%7D%3B%0A%20%20void%20print%28%29%7B%0A%20%20%20%20std%3A%3Acout%20%3C%3C%20zaehler%20%3C%3C%20%22/%22%20%3C%3C%20nenner%20%3C%3C%20std%3A%3Aendl%3B%0A%20%20%7D%0A%7D%3B%0A%0Aint%20main%28%29%7B%0A%20%20Bruch%20einhalb%20%7B1,2%7D%3B%0A%20%20Bruch*%20einviertel%20%3D%20static_cast%3CBruch*%3E%28malloc%28sizeof%28Bruch%29%29%29%3B%0A%20%20einviertel-%3Ezaehler%20%3D%201%3B%0A%20%20einviertel-%3Enenner%20%3D%204%3B%0A%20%20einviertel-%3Eprint%28%29%3B%0A%20%20Bruch*%20einachtel%20%3D%20new%20Bruch%281,8%29%3B%0A%20%20einachtel-%3Eprint%28%29%3B%0A%20%20return%20EXIT_SUCCESS%3B%0A%7D&curInstr=17&mode=display&origin=opt-frontend.js&py=cpp&rawInputLstJSON=%5B%5D)


**Zusammenfassung**

| Parameter                   | Stack                          | Heap                            |
| --------------------------- | ------------------------------ | ------------------------------- |
| Allokation und Deallokation | Automatisch durch den Compiler | Manuel durch den Programmierer  |
| Kosten                      | gering                         | ggf. höher durch Fragmentierung |
| Flexibilität                | feste Größe                    | Anpassungen möglich             |


**Und noch mal am Beispiel**

```cpp                     StackvsHeap.cpp
#include <iostream>

class MyClass{
  public:
    MyClass()
    {
      std::cout << "Constructor executed" << std::endl;
    }
    ~MyClass()
    {
      std::cout << "Deconstructor executed" << std::endl;
    }
};

int main()
{
  {  // Scope I
     MyClass A;
  }
  {  // Scope II
     MyClass* B = new MyClass;
  }
  std::cout << "Memory Leak !" << std::endl;
  return EXIT_SUCCESS;
}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

### Stack Overflow

Wenn ein Programm mehr Speicherplatz als die Stapelgröße belegt, tritt ein Stapelüberlauf auf und es kann zu einem Programmabsturz kommen. Es gibt zwei Fälle, in denen ein Stapelüberlauf auftreten kann:

1. Wenn wir eine große Anzahl lokaler Variablen deklarieren oder ein Array oder eine Matrix oder ein höherdimensionales Array mit großer Größe deklarieren, kann dies zu einem Überlauf des Stapels führen.

2. Wenn sich eine Funktion hinreichend  oft rekursiv selbst aufruft, kann der Stapel keine große Anzahl von lokalen Variablen speichern, die von jedem Funktionsaufruf verwendet werden, und führt zu einem Überlauf des Stapels.

Für Ubuntu 18.04 ergibt sich mit dem Befehl `ulimit` folgende Ausgabe:

```bash
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
```

Die Stackgröße ist **für diesen Rechner standardmäßig** auf 8192kB beschränkt. Wenn wir also einen Stack-Overflow generieren wollen
können wir dies realisieren, in dem wir Datenstrukturen generieren, die größer als dieser Wert sind.

$$
8192kB = 8192.000B = 1024.000 \cdot 8Byte
$$

```cpp            Stack OverFlow.cpp
#include <iostream>

int main(void)
{
    double array[1024*1024];
    //double* array = new double [1024*1024];
    std::cout << array[1000] << std::endl;
    std::cout << sizeof(double) << std::endl;
    // delete[] array;
    return EXIT_SUCCESS;
}
```

**Mögliche Lösungsansätze**

**Vermeidung**

+ Individuelle Analyse der statischen /dynamischen Codeelemente mit Hilfe des Compilers und Berücksichtigung der Aufrufhierarchie.

+ Testen mit vorinitialisiertem Speicher zur Bestimmung der maximalen Stack-Größe.

**Identifikation**

+ Verwendung von Stackpointer-Evaluationsregister (ARMv8-M Architekturen). Wenn der SP der CPU den in diesem Register festgelegten Wert (nennen wir es das SP_Limit-Register) unterschreitet (oder je nach Stack-Wachstum darüber liegt), wird eine Ausnahme generiert.

+ Die Memory Protection Unit (MPU) erlaubt die Integration von Sicherheitskorridoren zur Überwachung de Stackgröße.

+ Integration sogenannter "Canaries" als Muster im Speicher, die hinweise auf unberechtigte Schreiboperationen geben.

## (Raw-)Pointer / Referenzen
<!--
  comment: SelfDesignedUniquePointer.cpp
  ..............................................................................
  1.
     ```cpp
     int  i = 15;
     int& r = i;         // die Referenz r zeigt auf i.
     r = 17;             // i besitzt nun den Wert 17 statt 15.
     int& v;             // Fehler! v muss bei ihrer Deklaration
                         // initialisiert werden.
     ++r;                // Wert von i um Eins erhöhen.
     int* p = &r;        // Der Zeiger p erhält die Adresse von i, nicht von r!
     string s{"Text"};   // Erzeuge ein string-Objekt.
     string& z = s;
     z.erase();          // lösche den Inhalt von s.
     ```
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-->


C und C\+\+ unterstützen das Konzept des **Zeigers**, die sich von den meisten anderen Programmiersprachen unterscheiden. Andere Sprachen wie C#, C++(!), Java, Python etc. implementieren **Referenzen**.

Oberflächlich betrachtet sind sich Referenzen und Zeiger sehr ähnlich, in beiden
Fällen greifen wir indirekt auf einen Speicher zu:

+ Ein Zeiger ist eine Variable, die die Speicheradresse einer anderen Variablen enthält. Ein Zeiger muss mit dem Operator \* dereferenziert werden, um auf den Speicherort zuzugreifen. Pointer können auch ins "nichts" zeigen. C++11 definiert dafür den `nullptr`.

+ Eine Referenz ist ein Alias, das heißt ein anderer Name für eine bereits vorhandene Variable. Eine Referenz wird wie ein Zeiger implementiert, indem die Adresse eines Objekts gespeichert wird. Entsprechend kann eine Referenz auch nur für ein bestehendes Objekt angelegt werden. In der Nutzungsphase unterscheidet sich der Zugriff auf eine Referenz nicht von einem Direktzugriff

```cpp            Stack OverFlow.cpp
#include <iostream>

int main(void)
{
    int  i = 15;
    std::cout << i << std::endl;
    return EXIT_SUCCESS;
}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

Die Funktionsweise im Vergleich soll am Beispiel einer Parameterübergabe verdeutlicht werden:

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
  return EXIT_SUCCESS;
}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

**Nachteile bei der Verwendung von Raw Pointern**

> "Raw pointers have been a pain in the backside for most students learning C++ in the last decades. They had a multi-purpose role as raw memory iterators, nullable, changeable nullable references and devices to manage memory that no-one really owns. This lead to a host of bugs and vulnerabilities and headaches and decades of human life spent debugging, and the loss of joy in programming completely." [Link](https://arne-mertz.de/2018/04/raw-pointers-are-gone/)

> "In modernem C++ werden Rohzeiger nur in kleinen Codeblöcken mit begrenztem Gültigkeitsbereich, in Schleifen oder Hilfsfunktionen verwendet, in denen Leistung ausschlaggebend ist und keine Verwirrung über den Besitzer entstehen kann." [Microsoft](https://docs.microsoft.com/de-de/cpp/cpp/smart-pointers-modern-cpp?view=vs-2019)

## Smart Pointer
<!--
  comment: SelfDesignedUniquePointer.cpp
  ..............................................................................
  1.
     ```cpp
     class myPointer{
       private:
         MyClass* m_Ptr;
       public:
         myPointer(MyClass* ptr) : m_Ptr (ptr) {}
         ~myPointer(){
           delete m_Ptr;
         }
     };
     int main()
     {
       {
          //myPointer C(new MyClass());
          myPointer C = new MyClass();
       }
     }
     ```
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-->

> **Problemstellung:** Wir wollen den Komfort der Nutzung des Stacks auf dynamische Konzepte übertragen, die sich auf dem Heap befinden und mit Pointer adressiert werden.

Das folgende Beispiel greift einen Nachteil der Raw Pointer nochmals auf, um das
Konzept höherabstrakter Zeigerformate herzuleiten. Wie können wir sicherstellen,
dass im folgenden das Objekt auf dem Heap zerstört wird, wenn wir den Scope verlassen?

```cpp                     SelfDesignedUniquePointer.cpp
#include <iostream>

class MyClass{
  public:
    MyClass()
    {
      std::cout << "Constructor executed" << std::endl;
    }
    ~MyClass()
    {
      std::cout << "Deconstructor executed" << std::endl;
    }
};

int main()
{
  {
     MyClass* A = new MyClass();
     // ... Some fancy things happen here ...
     delete A;
  }
  return EXIT_SUCCESS;
}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

Wir kombinieren eine stackbasierten Pointerklasse, die die den Zugriff auf das
eigentliche Datenobjekt kapselt. Mit dem verlassen des Scopes wird auch der
allozierte Speicher freigegeben.

Visualisierung der Lösung in [PythonTutor](http://pythontutor.com/visualize.html#code=%23include%20%3Ciostream%3E%0A%0Aclass%20MyClass%7B%0A%20%20public%3A%0A%20%20%20%20MyClass%28%29%0A%20%20%20%20%7B%0A%20%20%20%20%20%20std%3A%3Acout%20%3C%3C%20%22Constructor%20executed%22%20%3C%3C%20std%3A%3Aendl%3B%0A%20%20%20%20%7D%0A%20%20%20%20~MyClass%28%29%0A%20%20%20%20%7B%0A%20%20%20%20%20%20std%3A%3Acout%20%3C%3C%20%22Deconstructor%20executed%22%20%3C%3C%20std%3A%3Aendl%3B%0A%20%20%20%20%7D%0A%7D%3B%0A%0Aclass%20MyPointer%7B%0A%20%20%20%20private%3A%0A%20%20%20%20%20%20%20%20MyClass*%20m_Ptr%3B%0A%20%20%20%20public%3A%0A%20%20%20%20%20%20%20%20MyPointer%28MyClass*%20ptr%29%20%3A%20m_Ptr%20%28ptr%29%20%7B%7D%0A%20%20%20%20%20%20%20%20~MyPointer%28%29%7B%0A%20%20%20%20%20%20%20%20%20%20%20%20delete%20m_Ptr%3B%0A%20%20%20%20%20%20%20%20%7D%0A%7D%3B%0A%0Aint%20main%28%29%0A%7B%0A%20%20%7B%0A%20%20%20%20%20MyPointer%20C%28new%20MyClass%28%29%29%3B%0A%20%20%7D%0A%20%20return%20EXIT_SUCCESS%3B%0A%7D&cumulative=false&heapPrimitives=nevernest&mode=edit&origin=opt-frontend.js&py=cpp&rawInputLstJSON=%5B%5D&textReferences=false)

Was sind die Schwächen des Entwurfes?

                                           {{1}}
********************************************************************************

```cpp
class MyPointer{
  private:
    MyClass* m_Ptr;
  public:
    MyPointer(MyClass* ptr) : m_Ptr (ptr) {}
    ~MyPointer(){
      delete m_Ptr;
    }
};

//myPointer C(new MyClass()); // oder
MyPointer C = new MyClass();
```

Die Wirkungsweise eines intelligenten C++-Zeigers ähnelt dem Vorgehen bei der Objekterstellung in Sprachen wie C#: Sie erstellen das Objekt und überlassen es dann dem System, das Objekt zur richtigen Zeit zu löschen. Der Unterschied besteht darin, dass im Hintergrund keine separate Speicherbereinigung ausgeführt wird – der Arbeitsspeicher wird durch die C++-Standardregeln für den Gültigkeitsbereich verwaltet, sodass die Laufzeitumgebung schneller und effizienter ist.

C++11 implementiert verschiedene Pointer-Klassen für unterschiedliche Zwecke. Diese werden im folgenden vorgestellt:

+ `std::unique_ptr` — smart pointer der den Zugriff auf eine dynamisch allokierte Ressource verwaltet.
+ `std::shared_ptr` — smart pointer der den Zugriff auf eine dynamisch allokierte Ressource verwaltet, wobei mehrere Instanzen für ein und die selbe Ressource bestehen können.
+ `std::weak_ptr` — analog zum `std::shared_ptr` aber ohne Überwachung der entsprechenden Pointerinstanzen.


********************************************************************************

### Unique Pointers
<!--
  comment: UniquePointer.cpp
  ..............................................................................
  1. Diskutiere den Sinn von
     ```cpp
      std::make_unique<MyClass>();  // exception save
      //std::unique_ptr<MyClass> A (new MyClass());
      //  1. Step - Generation of MyClass Instance
      //  2. Step - Generation of unique_ptr Instance
     ```
     Auf beiden Ebenen kann ein Fehler passieren, entweder generieren wir einen
     Pointer der ins Nichts zeigt oder eine Memory Leak.
  2. Versuch einer Kopieroperation
     ```cpp
     std:unique_ptr<MyClass> B = A;
     ```
     comment: UniquePointerHandling.cpp
     ..............................................................................
     Evaluaieren der Gültigkeit des Pointers
     if(!A){
       std::cout << "Invalid pointer!" << std::endl;
     }

    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-->

Die `unique` Pointer stellen sicher, dass das eigentliche Objekt nur durch einen
einzelnen Pointer adressiert wird. Wird der entsprechende Scope verlassen, wird das Konstrukt automatisch gelöscht.

```cpp                     UniquePointer.cpp
#include <iostream>
#include <memory>   //<-- Notwendiger Header

class MyClass{
  public:
    MyClass(){
      std::cout << "Constructor executed" << std::endl;
    }
    ~MyClass(){
      std::cout << "Deconstructor executed" << std::endl;
    }
    void print(){
      std::cout << "That's all!" << std::endl;
    }
};

int main()
{
  {
     std::unique_ptr<MyClass> A (new MyClass());
     A->print();
  }
  return EXIT_SUCCESS;
}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

> Merke: Zu jedem Zeitpunkt verweist nur eine Instanz des `unique_ptr` auf eine Ressource. Die Idee lässt keine Kopien zu.

Wie wird das softwaretechnisch abgefangen? Die entsprechenden Methoden wurden mit "delete" markiert und generieren entsprechend eine Fehlermeldung.

```cpp
unique_ptr(const _Myt&) = delete;
_Myt& operator=(const _Myt&) = delete;
```

Dies ist dann insbesondere bei der Übergabe von `unique_ptr` an Funktionen von Bedeutung.

```cpp    UniquePointerHandling.cpp
#include <iostream>
#include <memory>

void callByValue(std::unique_ptr<std::string> input){
    std::cout << *input << std::endl;
}

void callByReference(const std::unique_ptr<std::string> & input){
    std::cout << *input << std::endl;
}

void callByRawPointer(std::string* input){
    std::cout << *input << std::endl;
}

int main()
{
  // Variante 1
  //std::unique_ptr<std::string> A (new std::string("Hello World"))
  // Variante 2
  std::unique_ptr<std::string> A = std::make_unique<std::string>("Hello World");
  // Ohne Veränderung der Ownership
  callByValue(A);                   // Compilerfehler!!!
  //callByValue(std::move(A));      // Mit Übergabe der Ownership
  callByReference(A);
  callByRawPointer(A.get());
  //std::cout << A << std::endl;
  return EXIT_SUCCESS;
}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

Diese konzeptionelle Einschränkung bringt aber einen entscheidenden Vorteil mit sich. Der `unique` Pointer wird allein durch eine Adresse repräsentiert. Es ist kein weiterer Overhead für die Verwaltung des Konstrukts notwendig!
Vergleichen Sie dazu die Darstellung unter [cppreference](https://de.cppreference.com/w/cpp/memory/unique_ptr)


### Shared Pointers
<!--
  comment: SharedPointer.cpp
  ..............................................................................
  1. Füge einen neuen Scope ein, in dem verschiedene shared_ptr auf ein Objekt
     zeigen.
     ```cpp
     int main()
     {
        std::shared_ptr<MyClass> A;
       {
          std::shared_ptr<MyClass> B = make_shared<MyClass>();
          A = B;
       }
       A->print();
       return EXIT_SUCCESS;
     }
     ```
  2. Präsentiere die API des Shared POinter
     ```cpp
     B.use_count();
     B.unique();
     ```
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-->

std::shared_ptr sind intelligente Zeiger, die ein Objekt über einen Zeiger "verwalten". Mehrere shared_ptr Instanzen können das selbe Objekt besitzen. Das Objekt wird zerstört, wenn:

+ der letzte shared_ptr, der das Objekt besitzt, zerstört wird oder
+ dem letzten shared_ptr, der das Objekt besitzt, ein neues Objekt mittles operator= oder reset() zugewiesen wird.

Das Objekt wird entweder mittels einer delete-expression oder einem benutzerdefiniertem deleter zerstört, der dem shared_ptr während der Erzeugung übergeben wird.

```cpp                     SharedPointer.cpp
#include <iostream>
#include <memory>   //<-- Notwendiger Header

class MyClass{
  public:
    MyClass(){
      std::cout << "Constructor executed" << std::endl;
    }
    ~MyClass(){
      std::cout << "Deconstructor executed" << std::endl;
    }
    void print(){
      std::cout << "That's all!" << std::endl;
    }
};

int main()
{
  {
     std::shared_ptr<MyClass> A = std::make_shared<MyClass>();
     A->print();
  }
  std::cout << "Scope left" << std::endl;
  return EXIT_SUCCESS;
}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

Gegenwärtig sind Arrays noch etwas knifflig in der Representation und Handhabung. C++17 schafft hier erstmals die Möglichkeit einer adquaten Handhabung (vgl. StackOverflow Diskussionen).

Vergleiche [cppreference](https://en.cppreference.com/w/cpp/memory/shared_ptr) für die Nutzung der API.


### Weak Pointers

Ein `weak_ptr` ist im Grunde ein shared Pointer, der die Referenzanzahl nicht erhöht. Es ist definiert als ein intelligenter Zeiger, der eine nicht-besitzende Referenz oder eine schwache Referenz auf ein Objekt enthält, das von einem anderen shared Pointer verwaltet wird.

```cpp                     WeakPointer.cpp
#include <iostream>
#include <memory>

std::weak_ptr<int> gw;

void f()
{
    if (auto spt = gw.lock()) { // Has to be copied into a shared_ptr before usage
	  std::cout << *spt << "\n";
    }
    else {
        std::cout << "gw is expired\n";
    }
}

int main()
{
    {
      auto sp = std::make_shared<int>(42);
      gw = sp;
    	f();
    }
    f();
}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

### Regeln für den Einsatz von Pointern

Eine schöne Übersicht zu den Fehlern im Zusammenhang mit Smart Pointer ist unter
[https://www.acodersjourney.com](https://www.acodersjourney.com/top-10-dumb-mistakes-avoid-c-11-smart-pointers/)
zu finden:

+ _Using a shared pointer where an unique pointer suffices_
+ _Not making resources/objects shared by shared pointer threadsafe_
+ ...
+ _Not using `make_shared` to initialize a shared pointer_
+ ...

## Aufgabe der Woche

1. Machen Sie sich mit den wichtigsten gdb Mechanismen vertraut. Explorieren Sie den Stack eines Beispielprogrammes und versuchen Sie die Abläufe nachzuvollziehen.

2. Vergleichen Sie die Resultate der Codegrößen für verschiedene Optimierungsstufen. Gelingt es dem Compiler zum Beispiel eine einfache Aufgabe, die Sie geschickt strukturiert haben als solche zu identifizieren?

   Eine Hilfe dabei kann die Webseite [https://godbolt.org/](https://godbolt.org/)
   Zudem existiert eine ganze Reihe von youtube Filmen, die Beispiele diskutieren: [Link](https://www.youtube.com/watch?v=4_HL3PH4wDg)

3. Experimentieren Sie mit SmartPointern anhand von [https://thispointer.com/learning-shared_ptr-part-1-usage-details/](https://thispointer.com/learning-shared_ptr-part-1-usage-details/). Der Kurs fasst die Konzepte in 6 Präsentationen zusammen.
