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

Wie wird der Speicher von einem C++ Programm eigentlich verwaltet? Wie wird diese Struktur ausgehend vom Start eines Programmes aufgebaut?

<!--
style="width: 70%; max-width: 860px; display: block; margin-left: auto; margin-right: auto;"
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

```cpp
#include <iostream>

int main(void)
{
    return 0;
}
```

```bash
gcc memory.c -o memory
size memory
text       data        bss        dec        hex    filename
960        248         8          1216       4c0     memory
```

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


### Stack vs Heap

**Zunächst mal ganz praktisch, was passiert auf dem Stack?**

Ausgangspunkt unserer Untersuchung ist ein kleines Programm, das mit dem gnu Debugger `gdb` analysiert wurde:

```
g++ StackExample.cpp -o stackExample
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
}
```
@Rextester.CPP

Was passiert beim starten des Programmes und beim  Aufruf der Funktion `calc` "unter der Haube"? Schauen wir zunächst die Einrichtung des Stacks von Seiten der `main` funktion bis zur Zeile 12.

```
0x089d <+0>:  push   %rbp                 # Initalisierung des
                                          # main-Stackframes
0x089e <+1>:  mov    %rsp,%rbp            
0x08a1 <+4>:  sub    $0x10,%rsp           # Allokation der Variablen
0x08a5 <+8>:  movl   $0x11,-0xc(%rbp)     # Initialisierung
0x08ac <+15>: movl   $0x22,-0x8(%rbp)
0x08b3 <+22>: movl   $0x0,-0x4(%rbp)
0x08ba <+29>: mov    -0x8(%rbp),%edx      # Speichern der Parameter in
0x08bd <+32>: mov    -0xc(%rbp),%eax      # edx und eax
0x08c0 <+35>: mov    %edx,%esi
0x08c2 <+37>: mov    %eax,%edi           
0x08c4 <+39>: callq  0x88a <_Z4calcii>    # Aufruf der Funktion in Zeile 12
0x08c9 <+44>: mov    %eax,-0x4(%rbp)
0x08cc <+47>: mov    -0x4(%rbp),%eax
0x08cf <+50>: mov    %eax,%esi
0x08d1 <+52>: lea    0x200748(%rip),%rdi  # Aufruf der Betriebssystemfunktion für Ausgaben
0x08d8 <+59>: callq  0x760 <_ZNSolsEi@plt>
....
```

<!--
style="width: 100%; max-width: 860px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii
                                    kleinere Adresse

            +----------------> |                           |
            |                  +---------------------------+
            |          rbp-x10 | ????                      |
            |                  +---------------------------+
            |          rbp-x0c | num1                      |
      +------------+           +---------------------------+
rsb   |stackpointer|   rbp-x08 | num2                      |
      +------------+           +---------------------------+
                       rbp-x04 | result                    |   
      +------------+           +---------------------------+
rbp   |basepointer |------->   | Alter Framepointer        |
      +------------+           +---------------------------+       
                               | Alter Instruktionspointer |        
                               | ...                       |

                                     größere Adresse              
```

Nun rufen wir die Funktion `calc` auf und führen die Berechnung aus. Dafür  
wird ein neuer Stackframe angelegt. Wie entwickelt sich der Stack ausgehend von
dem zugehörigen Assemblercode weiter?

```
0x088a <+0>:  push   %rbp
0x088b <+1>:  mov    %rsp,%rbp
0x088e <+4>:  mov    %edi,-0x4(%rbp)
0x0891 <+7>:  mov    %esi,-0x8(%rbp)
0x0894 <+10>: mov    -0x4(%rbp),%eax
0x0897 <+13>: imul   -0x8(%rbp),%eax
0x089b <+17>: pop    %rbp
0x089c <+18>: retq
```


**Zusammenfassung**

| Parameter                   | Stack                          | Heap                           |
| --------------------------- | ------------------------------ | ------------------------------ |
| Allokation und Deallokation | Automatisch durch den Compiler | Manuel durch den Programmierer |
| Kosten                      | gering                         | höher                          |
| Flexibilität                | feste Größe                    | Anpassungen möglich                                |


### Stack Overflow

Wenn ein Programm mehr Speicherplatz als die Stapelgröße belegt, tritt ein Stapelüberlauf auf und es kann zu einem Programmabsturz kommen. Es gibt zwei Fälle, in denen ein Stapelüberlauf auftreten kann:

+ Wenn wir eine große Anzahl lokaler Variablen deklarieren oder ein Array oder eine Matrix oder ein höherdimensionales Array mit großer Größe deklarieren, kann dies zu einem Überlauf des Stapels führen.

+ Wenn sich eine Funktion hinreichend  oft rekursiv selbst aufruft, kann der Stapel keine große Anzahl von lokalen Variablen speichern, die von jedem Funktionsaufruf verwendet werden, und führt zu einem Überlauf des Stapels.

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

Die Stackgröße ist auf 8192kB beschränkt. Wenn wir also einen StackOverflow generieren wollen
können wir dies realisieren, in dem wir Datenstrukturen generieren, die größer als dieser Wert sind.

$$
8192kB = 8192.000B = 1024.000 \cdot 8Byte
$$

<Beispielrechnung>


## Wiederholung (Raw-)Pointer / Referenzen

**Was war noch mal eine Referenz?**

C und C\+\+ unterstützen das Konzept des **Zeigers**, die sich von den meisten anderen Programmiersprachen unterscheiden. Andere Sprachen wie C#, C++(!), Java, Python etc. implementieren **Referenzen**.

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
