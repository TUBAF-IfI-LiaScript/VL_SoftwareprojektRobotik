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

# Suche nach einer schönen Darstellung

<table>
    <thead>
        <tr>
          <th>Code</th>
          <th>text</th>
          <th>data</th>
          <th>bss</th>
          <th>dec</th>
        </tr>
    </thead>
    <tbody>
        <tr>
          <th><code><pre>
int main(void)
{
    return 0;
}
          </pre></code></th>
          <td>1918</td>
          <td>640</td>
          <td>8</td>
          <td>2566</td>
        </tr>
    </tbody>
</table>


## Stack vs Heap

> Ich möchte gern dieses Format übernehmen. Gibt es da einen css, den ich referenzieren kann

| Parameter                   | Stack                          | Heap                           |
| --------------------------- | ------------------------------ | ------------------------------ |
| Allokation und Deallokation | Automatisch durch den Compiler | Manuel durch den Programmierer |
| Kosten                      | gering                         | höher                          |
| Flexibilität                | feste Größe                    | Anpassungen möglich                                |



## Aufgabe der Woche

1.
