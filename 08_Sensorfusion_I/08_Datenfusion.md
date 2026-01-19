<!--

author:   Sebastian Zug & Georg Jäger
email:    sebastian.zug@informatik.tu-freiberg.de & Georg.Jaeger@informatik.tu-freiberg.de
version:  1.0.2
language: de
comment:  In dieser Vorlesungen werden die Grundkonzepte der Datenfusion adressiert.
narrator: Deutsch Female
attribute: thx

import:   https://github.com/liascript/CodeRunner
          https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/config.md

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/08_Datenfusion/08_Datenfusion.md)

# Datenfusion

<!-- data-type="none" -->
| Parameter            | Kursinformationen                                                                                             |
| -------------------- | ------------------------------------------------------------------------------------------------------------- |
| **Veranstaltung:**   | @config.lecture                                                                                               |
| **Semester**         | @config.semester                                                                                              |
| **Hochschule:**      | `Technische Universität Freiberg`                                                                             |
| **Inhalte:**         | `Grundlagen der Datenfusion für redundanter Sensoren`                                                         |
| **Link auf GitHub:** | https://github.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/blob/master/08_Datenfusion/08_Datenfusion.md |
| **Autoren**          | @author                                                                                                       |

![](https://media.giphy.com/media/665kTzjPTFGL9J2fNf/giphy-downsized.gif)

--------------------------------------------------------------------------------

**Zielstellung der heutigen Veranstaltung**

+ Vermittlung eines Grundverständnisses für die Datenfusion
+ Einführung eines Bayes basierten Schätzers als diskreten Filter

--------------------------------------------------------------------------------

## Wie weit waren wir gekommen?

... wir generieren ein "rohes" Distanzmessignal und haben es gefiltert.

<!--
style="width: 70%; max-width: 720px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii
                       +----------------------+
                       | Handlungsplanung     |   Strategie
                       +----------------------+
                                ^ ^ ^
                                | | |                  
                                v v v
                       +----------------------+
                       | Ausführung           |   Taktik
                       +----------------------+
                                ^ ^ ^
                                | | |
                                v v v
                       +----------------------+
                       | Reaktive Überwachung |   Ausführung
                       +----------------------+
 Sensordatenerfassung    ^ ^ ^          | | |    
 Aktuatoroperationen     | | |          v v v     
                       .-----------------------.
                       | Umgebung              |
                       .-----------------------.                                .
```

Im weiteren Verlauf der Veranstaltung werden wir uns auf den letzte Ebene fokussieren
und die elementare Verarbeitungskette verschiedener Sensorsysteme analysieren.



<!--
style="width: 70%; max-width: 7200px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii

       +----------+        +----------+                                
     +-+--------+ |     +--+--------+ |     +----------+     +----------+
+--> |Sensorik  +-+ --> | Filterung +-+ --> | Regelung | --> | Aktorik  | ---+
|    +----------+       +-----------+       +----------+     +----------+    |
|                                                                            |
|                              .---------------------.                       |
+----------------------------- | Umgebung            | <---------------------+
                               .---------------------.
```

--------------------------------------------------------------------------------

## Grundlagen und Konzepte

     {{0-1}}
*******************************************************************************
> Zielstellung: Übergreifende Abbildung der Ergebnisse einer multimodalen Umgebungserfassung

<!--
style="width: 80%; min-width: 420px; max-width: 720px;"
-->
```ascii
    +---------------------+    +---------------------+
    | Control Application |    | Control Application |
    +---------------------+    +---------------------+
               ^                   ^      ^      ^                             
               |                   |      |      |                             
    .---------------------.        |      |      |                             
    | Environment         |        |      |      |                            
    | representation      |        |      |      |                            
    .---------------------.        |      |      |                            
               ^                   |      |      |                             
               |                   |      |      |                             
    +---------------------+        |      |      |                             
    | Sensor Fusion       |        |      |      |                           
    +---------------------+        |      |      |                            
        ^      ^      ^            |      |      |                             
        |      |      |            |      |      |                             
      +---+  +---+  +---+        +---+  +---+  +---+  
      |S 0|  |S 1|  |S 2|        |S 0|  |S 1|  |S 2|   
    .---------------------.    .---------------------.
    | Environment         |    | Environment         |
```   
_Abbildung motiviert nach Wilfried Elmenreich, An Introduction to Sensor Fusion, Research Report 47/2001, TU Wien_

*******************************************************************************

      {{1-2}}
*******************************************************************************

Herausforderungen für die Fusion der Messdaten:

+ unterschiedliche Messraten (-> Synchronisation)
+ verschiedene räumliche Abdeckungen/ Auflösungen (-> Kalibrierung)
+ unterschiedliche (variable) Fehlermodelle
+ ...

Je nach Zielstellung verfolgt die Fusion unterschiedliche Ziele:

<!--
style="width: 80%; min-width: 420px; max-width: 720px;"
-->
```ascii

Ziel             Zuverlässigkeit     Vollständigkeit      erweiterter
                 Genauigkeit                              Erfassungsbereich

Resultierende    Objekt A            Objekt A+B           Objekt C
Daten                ^                    ^                   ^     
                     |                    |                   |
               +---------------+    +---------------+     +---------------+  
Fusion         | Konkurrierend |    | Komplementär  |     | Kooperativ    |
               +---------------+    +---------------+     +---------------+  
                 ^            ^       ^         ^            ^          ^      
                 |             \     /          |            |          |      
                 |              \   /           |            |          |      
               .---.            .---.         .---.        .---.      .---.   
Sensoren       |S 0|            |S 1|         |S 2|        |S 3|      |S 4|
                  .             .               .             .         .
                   '.         .'                .              '.     .'
Environment           .-----.                +-----+              .-.
                       \ A /                 |  B  |             ( C )
                        \ /                  +-----+              '-'
                         .
```   
_Unterschiedliche Fusionsansätze analog zu Brooks und Iyengar (1997)_

+ Bei der __konkurrierenden Fusion__ erfassen Sensoren gleichzeitig denselben Sichtbereich und liefern Daten gleicher Art. Die (oft gewichtete) Verknüpfung solcher, "konkurrierender" Daten kann die Genauigkeit des Gesamtsystems erhöhen.

+ Eine __komplementäre Fusion__ hat das Ziel, die Vollständigkeit der Daten zu erhöhen. Unabhängige Sensoren betrachten hierfür unterschiedliche Sichtbereiche und Phänomene oder messen zu unterschiedlichen Zeiten.

+ Reale Sensoren erbringen die gewünschten Informationen oft nicht allein. So ergibt sich beispielsweise die benötigte Information erst aus dem Zusammensetzen der verschiedenen Ausgabedaten. Eine solche Fusion wird als __kooperative Fusion__ bezeichnet.

*******************************************************************************

        {{2-3}}
*******************************************************************************

Was aber wird fusioniert? Ein einfaches Distanzmaß, die Positioninformation eines Roboters oder die Aufenthaltswahrscheinlichkeit in einem bestimmten Raum ... Hall & Llinas (1997) unterscheiden dafür drei Ebenen der Sensordatenfusion, die die Datenkategorien - Raw Data, Feature und Decicion - referenzieren:

+ Bei der __data fusion__ werden die rohen Sensordaten vor weiteren Signalverarbeitungsschritten miteinander verschmolzen.
+ Bei der __feature fusion__ erfolgt vor der Verschmelzung eine Extraktion eindeutiger Merkmale. Die neu kombinierten Merkmalsvektoren werden im Anschluss weiterverarbeitet.
+ Bei der __decision fusion__ erfolgt die Zusammenführung erst nachdem alle Signalverarbeitungs- und Mustererkennungsschritte durchgeführt wurden.

*******************************************************************************

       {{3-4}}
*******************************************************************************

Ein immer wieder zitiertes Architekturkonzept für die Abbildung dieser Ansätze ist das _Joint Directors of Laboratories_ (JDL) Modell.

![ImageMatching](./images/JDLmodell.png)<!-- style="width: 70%;"-->

_Sensor and Data Fusion: Concepts and Application, second ed., vol. TT14SPIE Press (1999)_

1. Vorverarbeitung : zeitliche und räumliche Registrierung der Daten sowie Vorverarbeitungsmaßnahmen auf Signal- oder Pixelebene.
2. Objekterkennung und Extraktion Schätzung und Vorhersage von kontinuierlichen oder diskreten Objektmerkmalen.
3. Situationsanalyse: alle detektierten Objekte werden in einen größeren Kontext gebracht und Objektbeziehungen analysiert werden.
4. Bedrohungsanalyse: Situationsabhängig werden, im Sinne einer Risikominimierung, unterschiedliche Handlungsoptionen evaluiert.
5. Prozessoptimierung: Verwaltung und Adaption der Ressourcen

*******************************************************************************

## Einführung in den diskreten Bayes Filter

Das Beispiel basiert auf der hervorragenden Einführung in die Mathematik und Anwendung von Fusionsansätzen des

https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/02-Discrete-Bayes.ipynb


Das Anwendungsbeispiel zielt auf den Schwimmroboter, der sich auf der Freiberger Mulde bewegt, lässt sich aber analog auf beliebige andere 1D, 2D oder 3D Szenarien übertragen.

![ImageMatching](./images/mira.jpg)<!-- style="width: 70%;"-->


Anstatt eine kontinuierliche Messung zu realisieren bilden wir unsere Position auf diskrete Streckenabschnitte ab. Im Bild oben werden diese mit 0-9 indiziert.

```ascii

  Streckensegmente der Freiberger Mulde

| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
======O===SB======O===O===B===S=======S== -->
                                          Strömung
                                                                                                                                              .
```   
_Abbildung des Streckenmodels (O = Orange Warnschilder am Ufer, S = Starke Strömung, B = Brücken)_

Welche Anfangskenntnis zur Position haben wir zunächst? Die Aufenthaltswahrscheinlichkeit ist für alle 10 Streckensegmente gleich und entsprechend $p_i=0.1$ für alle $0 \leq i < 10$

```python                          generateBelief.py
import numpy as np
import matplotlib.pyplot as plt
belief = np.array([1./10]*10)
print(belief)

fig, ax = plt.subplots(figsize=(8,4))
ax.bar(np.arange(len(belief)), belief)
plt.ylim(0, 1)
#plt.show()  
plt.savefig('foo.png') # notwendig für die Ausgabe in LiaScript
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)


Es wird deutlich, dass wir aktuell noch kein Wissen um die Position des Bootes haben. Man spricht vom "apriori-"Wissen.

### Sensoren

Unser Roboter ist mit verschiedenen Sensorsystemen ausgestattet.

1. Im Frontbereich des Roboters befindet sich ein Kamerasystem, dass aus Bilddaten Features zu erkennen versucht. Dabei konzentrierte man sich auf orange Warnschilder und Brücken über die Mulde.

2. Ein Beschleunigungssensor bestimmt die Bewegungen des Bootes. Damit lassen sich zum Beispiel starke Strömungen gut erkennen.

3. An einige Stellen können mit dem GNSS-System Messdaten generiert werden. Die umgebenden Höhenzüge behindern dies.

Alle drei Messmethoden sind mit Fehlern überlagert. Welche können das sein?

### Bildsensor

Nun nutzen wir den ersten Sensormodus, die Erkennung der orangen Zeichen. Wir gehen davon aus, dass der Sensor nur in seltenen Fällen eine falsche Klassifikation (Schild/kein Schild) vornimmt.

```ascii

                   Streckensegmente der Freiberger Mulde

                 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
                 ======O===SB======O===O===B===S=======S== -->
                                                           Strömung            
                                                                                                                                              .
```   
_Abbildung des Streckenmodels (O = Orange Warnschilder am Ufer, S = Starke Strömung, B = Brücken)_

Im folgenden sprechen wir vom tatsächlichen Zustand $x$ und der Messung $z$. Für unsere Messmethode haben wir verschiedene Laborexperiemente gemacht und eine Vierfelder-Tafel erstellt.

<!--
style="width: 100%; min-width: 380px; max-width: 720px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii
                               Schild im Segement
                                    ja    nein
                                 +------+------+
                           ja    | 10   |   2  |  12                          
           Schild erkannt        +------+------+
                         nein    | 1    |   7  |   8
                                 +------+------+
                                   11       9     20
```  

*Vierfeldertafel unseres "Schilderkenners"*

Abgebildet auf Wahrscheinlichkeiten bedeutet das ja:

<!--
style="width: 100%; min-width: 380px; max-width: 720px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii
                               Schild im Segement
                                    ja    nein
                                 +------+------+
                           ja    | 0.5  | 0.1  |   0.6                         
           Schild erkannt        +------+------+
                         nein    | 0.05 | 0.35 |   0.4
                                 +------+------+
                                   0.55   0.45     1.0
```  

Wie wahrscheinlich ist es also, dass wir uns tatsächlich an einem orangen Schild
befinden, wenn wir eine entsprechende Messung vorliegen haben?

$$p(x|z) = \frac{0.5}{0.5 + 0.1} = 0.83$$

Wie wahrscheinlich ist eine Messung eines orangen Schildes, wenn wir gar keines erreicht haben?

$$p(x|z) = \frac{0.1}{0.5 + 0.1} = 0.17$$

Wie können wir diese Sensorcharakteristik nun für unser Roboterbeispiel verwenden? Unsere Klassifikation wird durch die verschiedenen Möglichkeiten, an denen sich der Roboter aufhalten kann, „verwässert“. Wenn wir nur ein Segment mit einem Schild hätten müssten wir diesem eine Aufenthaltswahrscheinlichkeit von $p=0.83$ zuordnen. Da es aber mehrere Möglichkeiten gibt, splittet sich diese auf.

```python                          generateMeasurements.py
import numpy as np
import matplotlib.pyplot as plt

markers = np.array([0, 1., 0, 0, 1., 1., 0, 0, 0, 0])
truePositive = 0.83
belief = markers / sum(markers)*truePositive
belief[markers == 0] = (1-truePositive)/np.count_nonzero(markers==0)
print(belief)

fig, ax = plt.subplots(figsize=(8,4))
ax.bar(np.arange(len(belief)), belief)
plt.ylim(0, 1)
#plt.show()  
plt.savefig('foo.png') # notwendig für die Ausgabe in LiaScript
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)

### Abbildung auf apriori Wissen

Was bedeutet dies aber im Hinblick auf die Positionsbestimmung unter Berücksichtigung des apriori-Wissens?

Wir haben zwei Vektoren mit Wahrscheinlichkeiten bezüglich unseres Aufenthaltes. Die Kombination draus ist eine einfaches Produkt gefolgt von einer Normalisierung.

$$posteriori = \frac{belief \cdot prior}{normalization}$$


```python                          generateBelief.py
import numpy as np
import matplotlib.pyplot as plt

# Vorwissen der Position
apriori = np.array([1./10]*10)
# Messungen
markers = np.array([0, 1., 0, 0, 1., 1., 0, 0, 0, 0])
truePositive = 0.83
belief = markers / sum(markers)*truePositive
belief[markers == 0] = (1-truePositive)/np.count_nonzero(markers==0)
posteriori = (apriori * belief) / sum(apriori*belief)

fig, ax = plt.subplots(figsize=(8,4))
width = 0.35
ax.bar(np.arange(len(apriori)), apriori, width)
ax.bar(np.arange(len(posteriori)) + width, posteriori, width)
plt.ylim(0, 1)
#plt.show()  
plt.savefig('foo.png') # notwendig für die Ausgabe in LiaScript
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)

Die Positionen der orangen Schilder heben sich deutlich ab, obwohl wir bei der konkreten Erkennung unsicher sind. 
Welche Veränderung erwarten Sie, wenn wir die Qualität der Sensormessungen erhöhen?

Wie können wir unsere Positionsschätzung verbessern:

1. Verbesserung der Qualität der Sensoren
2. Einbettung weiterer Sensoren
3. Wiederholung der Messungen (sofern wir von statistisch unabhängigen Messungen ausgehen)

Lassen Sie uns die Messung einige Male wiederholen. Das folgende Diagramm zeigt die Schätzung an der Position 1, nachdem mehrere Messungen fusioniert wurden.

```python                          constantPosition.py
import numpy as np
import matplotlib.pyplot as plt

apriori = np.array([1./10]*10)
markers = np.array([0, 1., 0, 0, 1., 1., 0, 0, 0, 0])
truePositive = 0.83
belief = markers / sum(markers)*truePositive
belief[markers == 0] = (1-truePositive)/np.count_nonzero(markers==0)
p_1 = []; p_1.append(apriori[1])
posteriori = apriori
for i in range(1, 10):
  posteriori = (posteriori * belief) / sum(posteriori*belief)
  p_1.append(posteriori[1])
print(p_1)

fig, ax = plt.subplots(figsize=(8,4))
ax.bar(np.arange(len(p_1)), p_1)
plt.ylim(0, 1)
#plt.show()  
plt.savefig('foo.png') # notwendig für die Ausgabe in LiaScript
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)

Unsere Positionsschätzung nähert sich der belief-Verteilung unsere Messung (3 von 10 Positionen sind mit einem orangen Schild ausgestattet) an. Der Einfluß des Anfangswissens geht zurück. Je häufiger ich messe, desto mehr vertraue ich den Messungen, sofern diese statistisch unabhängig sind.

### Bewegung

Die über der Zeit gestiegene Qualität der Positionsschätzung lässt sich bisher nur realisieren, wenn keine Bewegung erfolgt. Wie aber kann diese abgebildet werden?

Nehmen wir wiederum eine Aufenthaltwahrscheinlichkeit über unseren Segmenten an, die durch eine Bewegung $u$ verändert wird. $u$ ist dabei die Kontrollfunktion, mit der wir unserer Umgebung/Roboter manipulieren Der Einfachheit halber berücksichtigen wir zunächst eine ungestörte Abbildung. Für $u=v=1$ verschiebt sich unser Roboter um eine Einheit.

```python                          movements.py
import numpy as np
import matplotlib.pyplot as plt

def perfect_predict(belief, u):
    n = len(belief)
    result = np.zeros(n)
    for i in range(n):
        result[i] = belief[(i-u) % n]
    return result

belief = np.array([0.05, .35, .1, .2, .3, 0, 0, 0, 0, 0, 0])
result = perfect_predict(belief, 1)

fig, ax = plt.subplots(figsize=(8,4))
width = 0.35
ax.bar(np.arange(len(belief)), belief, width)
ax.bar(np.arange(len(result)) + width, result, width)
plt.ylim(0, 1)
#plt.show()  
plt.savefig('foo.png') # notwendig für die Ausgabe in LiaScript
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)

Welche Probleme sehen Sie?

Allerdings ist es völlig unrealistisch, dass sich unser System perfekt verhält. Strömungen und Dynamikeinflüsse generieren situative Verstärkungen oder Abschwächungen der Geschwindigkeit.

Zunächst berücksichtigen nur die beiden benachbarten Felder und ein perfektes apriori Wissen. Durch die Strömung bedingt verlässt unser Roboter die Position 2 trotz einer Beschleunigung stromab nicht, möglicherweise bewegt er sich aber auch zwei Segmente weiter.

```python                          uncertainmovements.py
import numpy as np
import matplotlib.pyplot as plt

def predict_move(belief, move, p_under, p_correct, p_over):
    n = len(belief)
    prior = np.zeros(n)
    for i in range(n):
        prior[i] = (
            belief[(i-move) % n]   * p_correct +
            belief[(i-move-1) % n] * p_over +
            belief[(i-move+1) % n] * p_under)      
    return prior

belief = [0., 0., 1. , 0. , 0., 0., 0., 0., 0., 0.]
#belief = [0., 0.,  .4,  .6, 0., 0., 0., 0., 0., 0.]
result = predict_move(belief, 1, .1, .7, .2)

fig, ax = plt.subplots(figsize=(8,4))
width = 0.35
ax.bar(np.arange(len(belief)), belief, width)
ax.bar(np.arange(len(result)) + width, result, width)
plt.ylim(0, 1)
#plt.show()  
plt.savefig('foo.png') # notwendig für die Ausgabe in LiaScript
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)

Was aber geschieht, wenn wir von einem unsicheren priori Wissen ausgehen?

```
//        0   1    2   3   4   5   6   7   8   9
belief = [0., 0., .4, .6,  0., 0., 0., 0., 0., 0.]
```

Wie groß ist die Wahrscheinlichkeit, dass wir Segment x erreichen? Dabei berücksichtigen wir nun die verschiedenen Startpunkte, in dem wir die Abbildungsfunktion von $u$ auf alle Kombinationen anwenden.

<!-- data-type="none" -->
| Segment | Rechnung                        | Ergebnis         |
| ------- | ------------------------------- | ---------------- |
| 0       |                                 | nicht erreichbar |
| 1       |                                 | nicht erreichbar |
| 2       | $0.4 \cdot 0.1$                 | $0.04$           |
| 3       | $0.4 \cdot 0.7 + 0.6 \cdot 0.1$ | $0.34$           |
| 4       | $0.4 \cdot 0.2 + 0.6 \cdot 0.7$ | $0.5$            |
| 5       | $0.6 \cdot 0.2$                 | $0.12$           |
| 6       |                                 | nicht erreichbar |
| 7       |                                 | ...              |

Grafisch dargestellt ergibt sich damit folgendes Bild:

<!--
style="width: 100%; min-width: 380px; max-width: 920px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii
             |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  |

    priori      0     0    0.4   0.6    0     0     0     0     0     0      

Reihenfolge   [0.2   0.7   0.1]  An die Stelle i=2 kann ich von i=0 (p=0.2)  
invertiert                 0.04  und i=1 (p=0.7) gelangen.        
                           ====

                    [0.2   0.8   0.1]  
                                 0.34        
                                 ====

                           [0.2   0.8   0.1]  
                                        0.5
                                        ====
```   
_Abbildung des Streckenmodels (O = Orange Warnschilder am Ufer, S = Starke Strömung, B = Brücken)_

Damit bilden wir eine diskrete Faltungsoperation über unserer Aufenthaltswahrscheinlichkeit ab. Die Defintion der Faltung ergibt sich zu

$$(f * g)(n) = \sum_{k \in D} f(k) g(n - k)$$

Wenn wir die Rechung also verallgemeinern können wir auf eine bestehende Implementierung zurückgreifen. Die `scipy` Bibliothek hält eine Funktion `convolve` bereit [Link](https://docs.scipy.org/doc/scipy/reference/generated/scipy.ndimage.convolve.html)


```python                          uncertainmovements.py
import numpy as np
from scipy import ndimage
import matplotlib.pyplot as plt

belief = [.05, .5, .05, .05, .05, .05, .05, .05, .05, .05]
kernel = [.1, 0.7, 0.2]
prior = ndimage.convolve(np.roll(belief, int(len(kernel) / 2)), kernel, mode='wrap')

#belief = prior   # Multiple movements
#prior = ndimage.convolve(np.roll(belief, int(len(kernel) / 2)), kernel, mode='wrap')

fig, ax = plt.subplots(figsize=(8,4))
width = 0.35
ax.bar(np.arange(len(belief)), belief, width)
ax.bar(np.arange(len(prior)) + width, prior, width)
plt.ylim(0, 1)
#plt.show()  
plt.savefig('foo.png') # notwendig für die Ausgabe in LiaScript
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)


Mit jedem Prädiktionsschritt fächert die Breite der Unsicherheit entsprechend auf.

### Und jetzt alles zusammen

Fassen wir nun beide Aspekte, die Vorhersage des Systemverhaltens und die Korrektur anhand der Messdaten zusammen. Aus dieser Konstallation wird deutlich, dass wir einen iterativen Prozess realisieren, in dessen Ablauf eine Zustandsvariable, hier unser Positionsindex "verfolgt" wird.

| Zustand         | Bedeutung                                                                                              |
| --------------- | ------------------------------------------------------------------------------------------------------ |
| Initialisierung | Definition einer Anfangsschätzung                                                                      |
| Vorhersage      | Auf der Grundlage des unsicheren Systemverhaltens den Zustand für den nächsten Zeitschritt vorhersagen |
| Aktualisierung  | Eintreffen einer Messung, Abbildung auf die Wahrscheinlichkeit eines Systemzustandes                   |


<!--
style="width: 100%; min-width: 380px; max-width: 720px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii

       Inital               Measurements
    \  Conditions               /
     v                         v                                              
    .-------.  ------->  .-------.
   ( Predict )          ( Update  )        
    `-------'  <-------  `-------'
       |
       v  State
          Estimation
```

Das folgende Codefragment bildet zwei Iterationen für unser Beispiel ab. Im ersten Durchlauf ändert die Prediktionsphase den intertialen Wissensstand nicht. Die Faltung des Kernels ändert die Aufenthaltwahrscheinlichkeit nicht. Eine Präzisierung erfährt diese mit der ersten Messung durch den Schildersensor.

```python                          BayesFilter.py
import numpy as np
from scipy import ndimage
import matplotlib.pyplot as plt

priori = np.array([1./10]*10)
kernel = [.1, 0.7, 0.2]
markers = np.array([0, 1., 0, 0, 1., 1., 0, 0, 0, 0])
truePositive = 0.83
belief = markers / sum(markers)*truePositive
belief[markers == 0] = (1-truePositive)/np.count_nonzero(markers==0)

# FIRST LOOP - prediction
posteriori = ndimage.convolve(np.roll(priori, int(len(kernel) / 2)), kernel, mode='wrap')
# FIRST LOOP - update
priori = posteriori * belief / sum(posteriori * belief )

# SECOND LOOP - prediction
posteriori = ndimage.convolve(np.roll(priori, int(len(kernel) / 2)), kernel, mode='wrap')
# SECOND LOOP - update
priori = posteriori * belief / sum(posteriori * belief )

fig, ax = plt.subplots(figsize=(8,4))
width = 0.35
ax.bar(np.arange(len(priori)), priori, width)
ax.bar(np.arange(len(posteriori)) + width, posteriori, width)
plt.ylim(0, 1)
#plt.show()  
plt.savefig('foo.png') # notwendig für die Ausgabe in LiaScript
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)

### Erweiterung auf 2D: Roboter-Lokalisierung im Grid

--{{0}}--
Nach dem eindimensionalen Schwimmroboter-Beispiel wollen wir nun den diskreten Bayes-Filter auf ein realistischeres zweidimensionales Szenario übertragen. Dabei werden wir sehen, dass die gleichen Prinzipien gelten, aber die Komplexität deutlich zunimmt. Stellen Sie sich einen mobilen Roboter vor, der sich in einer Fabrikhalle oder einem Bürogebäude orientieren muss.

Das 1D-Beispiel mit dem Schwimmroboter hat die Grundprinzipien verdeutlicht. Für realistische Anwendungen müssen wir den Filter jedoch auf zwei oder mehr Dimensionen erweitern. Betrachten wir einen mobilen Roboter, der sich in einer 2D-Umgebung bewegt und Landmarken erkennen kann.

<!--
style="width: 80%; min-width: 420px; max-width: 720px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii
    Spalte  0   1   2   3   4   5   6   7   8   9
          +---+---+---+---+---+---+---+---+---+---+
Zeile 0   |   |   | L |   |   |   |   | L |   |   |
          +---+---+---+---+---+---+---+---+---+---+
      1   |   |   |   |   |   |   |   |   |   |   |
          +---+---+---+---+---+---+---+---+---+---+
      2   | L |   |   |   |   |   |   |   |   | L |
          +---+---+---+---+---+---+---+---+---+---+
      3   |   |   |   |   |   |   |   |   |   |   |
          +---+---+---+---+---+---+---+---+---+---+
      4   |   | L |   |   |   |   | L |   |   |   |
          +---+---+---+---+---+---+---+---+---+---+
      5   |   |   |   |   | R→|   |   |   |   |   |
          +---+---+---+---+---+---+---+---+---+---+

    L = Landmarke (z.B. QR-Code, ArUco-Marker)
    R = Roboter (wahre Position)
```

**Szenario**: Ein mobiler Roboter soll sich in einer bekannten Umgebung lokalisieren.

> **Was weiß der Roboter?**
>
> | Information | Bekannt? | Quelle |
> |-------------|----------|--------|
> | Karte der Umgebung (Landmarken-Positionen) | Ja | A-priori Wissen |
> | Eigene Orientierung (Blickrichtung) | Ja | Kompass / Gyroskop |
> | Bewegungskommandos ("fahre 1m nach Osten") | Ja | Odometrie |
> | **Absolute Position** | **Nein** | *Das schätzen wir!* |
> | Welche Landmarke erkannt wurde (ID) | Nein | Sensor liefert nur "Landmarke ja/nein" |

Der Roboter verfügt über:

1. **Kamera**: Erkennt, *ob* eine Landmarke in Sichtweite ist, aber nicht *welche* (keine Unterscheidung der Marker)
2. **Odometrie + Kompass**: Liefert Bewegungsrichtung und -distanz (mit Unsicherheit)

Diese Konstellation ist typisch für einfache Indoor-Roboter und verdeutlicht das Lokalisierungsproblem: Der Roboter muss aus mehrdeutigen Messungen (mehrere Landmarken sehen gleich aus) und unsicheren Bewegungen seine Position eingrenzen.

--{{0}}--
Beachten Sie die wichtige Annahme: Der Roboter kennt seine Orientierung durch einen Kompass oder ein Gyroskop. Er weiß also, in welche Himmelsrichtung er schaut und fährt. Was er nicht weiß, ist seine absolute Position im Raum. Das ist genau das, was wir mit dem Bayes-Filter schätzen wollen. Die Landmarken sind dabei absichtlich nicht unterscheidbar - der Roboter sieht nur "da ist ein Marker", aber nicht welcher. Das macht das Problem interessant, denn eine einzelne Messung ist mehrdeutig.

       {{0-1}}
*******************************************************************************

**Schritt 1: Weltmodell und initiale Belief**

--{{0}}--
Im ersten Schritt modellieren wir die Umgebung als diskretes Gitter. Jede Zelle repräsentiert einen möglichen Aufenthaltsort des Roboters. Die acht Landmarken sind an festen Positionen platziert, die dem Roboter aus einer Karte bekannt sind. Die initiale Belief-Verteilung ist uniform: Da wir zu Beginn keine Information über die Position haben, ist jede der 80 Zellen gleich wahrscheinlich mit einer Wahrscheinlichkeit von 1,25 Prozent.

Zunächst definieren wir unsere Welt und die Anfangsverteilung. Der Roboter weiß initial nicht, wo er sich befindet - alle Positionen sind gleich wahrscheinlich.

```python                          World2D.py
import numpy as np
import matplotlib.pyplot as plt

# Weltgröße
ROWS, COLS = 8, 10

# Landmarken-Positionen (Zeile, Spalte)
landmarks = [(0, 2), (0, 7), (2, 0), (2, 9), (4, 1), (4, 6), (6, 3), (6, 8)]

# Weltmodell erstellen
world = np.zeros((ROWS, COLS))
for (r, c) in landmarks:
    world[r, c] = 1

# Initiale Belief: Gleichverteilung
belief = np.ones((ROWS, COLS)) / (ROWS * COLS)

# Visualisierung
fig, axes = plt.subplots(1, 2, figsize=(12, 5))

# Weltmodell
ax1 = axes[0]
ax1.imshow(world, cmap='Greens', alpha=0.5)
for (r, c) in landmarks:
    ax1.plot(c, r, 'g^', markersize=15, markeredgecolor='darkgreen', markeredgewidth=2)
ax1.set_title('Weltmodell mit Landmarken')
ax1.set_xlabel('Spalte')
ax1.set_ylabel('Zeile')

# Initiale Belief
ax2 = axes[1]
im = ax2.imshow(belief, cmap='Blues', vmin=0, vmax=0.1)
ax2.set_title(f'Initiale Belief (uniform: {belief[0,0]:.4f})')
ax2.set_xlabel('Spalte')
ax2.set_ylabel('Zeile')
plt.colorbar(im, ax=ax2, label='P(Position)')

plt.tight_layout()
plt.savefig('foo.png')
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)

*******************************************************************************

       {{1-2}}
*******************************************************************************

**Schritt 2: Sensormodell - Measurement Update**

--{{1}}--
Jetzt kommt der spannende Teil: Der Roboter macht seine erste Messung und erkennt eine Landmarke! Aber welche? Das weiß er nicht. Trotzdem können wir diese Information nutzen. Wir modellieren den Sensor mit zwei Wahrscheinlichkeiten: Die True-Positive-Rate von 85 Prozent sagt uns, wie zuverlässig der Sensor eine tatsächlich vorhandene Landmarke erkennt. Die False-Positive-Rate von 10 Prozent beschreibt, wie oft der Sensor fälschlicherweise eine Landmarke meldet, obwohl keine da ist. Mit dem Satz von Bayes können wir nun die Belief aktualisieren: An Positionen mit Landmarken steigt die Wahrscheinlichkeit deutlich an, während sie an allen anderen Positionen sinkt.

Der Roboter erkennt eine Landmarke. Wie aktualisieren wir die Belief?

+ $P(\text{Erkennung} | \text{Landmarke vorhanden}) = 0.85$ (True Positive)
+ $P(\text{Erkennung} | \text{keine Landmarke}) = 0.1$ (False Positive)

```python                          MeasurementUpdate2D.py
import numpy as np
import matplotlib.pyplot as plt

ROWS, COLS = 8, 10
landmarks = [(0, 2), (0, 7), (2, 0), (2, 9), (4, 1), (4, 6), (6, 3), (6, 8)]

world = np.zeros((ROWS, COLS))
for (r, c) in landmarks:
    world[r, c] = 1

def measurement_update(belief, world, detected_landmark, p_hit=0.85, p_miss=0.1):
    """Update der Belief nach Sensormessung"""
    if detected_landmark:
        # Likelihood: höher an Landmarken-Positionen
        likelihood = np.where(world == 1, p_hit, p_miss)
    else:
        # Keine Landmarke erkannt
        likelihood = np.where(world == 1, 1 - p_hit, 1 - p_miss)

    # Bayes-Update
    posterior = belief * likelihood
    return posterior / np.sum(posterior)  # Normalisierung

# Initiale Belief
belief = np.ones((ROWS, COLS)) / (ROWS * COLS)

# Roboter erkennt eine Landmarke!
belief_after = measurement_update(belief, world, detected_landmark=True)

# Visualisierung
fig, axes = plt.subplots(1, 2, figsize=(12, 5))

ax1 = axes[0]
im1 = ax1.imshow(belief, cmap='Blues', vmin=0, vmax=np.max(belief_after))
for (r, c) in landmarks:
    ax1.plot(c, r, 'g^', markersize=12)
ax1.set_title('Vor der Messung')
plt.colorbar(im1, ax=ax1)

ax2 = axes[1]
im2 = ax2.imshow(belief_after, cmap='Blues', vmin=0, vmax=np.max(belief_after))
for (r, c) in landmarks:
    ax2.plot(c, r, 'g^', markersize=12)
ax2.set_title('Nach Messung: Landmarke erkannt!')
plt.colorbar(im2, ax=ax2)

plt.tight_layout()
plt.savefig('foo.png')

print(f"Max. Belief vorher:  {np.max(belief):.4f}")
print(f"Max. Belief nachher: {np.max(belief_after):.4f}")
print(f"Summe der Belief an Landmarken: {np.sum(belief_after[world==1]):.4f}")
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)

--{{1}}--
In der Visualisierung sehen Sie den Effekt: Die vorher gleichmäßig blaue Heatmap zeigt jetzt deutliche Spitzen an den acht Landmarken-Positionen. Die Summe der Belief an diesen Positionen beträgt nun fast 50 Prozent, obwohl sie nur 10 Prozent der Fläche ausmachen. Der Roboter hat also bereits nach einer einzigen Messung wertvolle Information gewonnen, auch wenn er noch nicht eindeutig lokalisiert ist.

Die Wahrscheinlichkeit konzentriert sich nun auf die Landmarken-Positionen!

*******************************************************************************

       {{2-3}}
*******************************************************************************

**Schritt 3: Bewegungsmodell - Motion Update (Predict)**

--{{2}}--
Nun bewegt sich der Roboter. Er erhält den Befehl, eine Zelle nach Osten zu fahren. Aber Roboterbewegungen sind nie perfekt! Radschlupf, unebener Boden oder ungenaue Motoren führen dazu, dass die tatsächliche Bewegung von der gewünschten abweicht. Wir modellieren das mit einem Bewegungskernel: Mit 70 Prozent Wahrscheinlichkeit landet der Roboter dort, wo er hin wollte. Mit jeweils 10 Prozent driftet er nach Nordost oder Südost ab. Und mit 10 Prozent bleibt er stecken und bewegt sich gar nicht. Mathematisch realisieren wir das durch eine zweidimensionale Faltung der Belief-Verteilung mit diesem Kernel.

Der Roboter bewegt sich nach Osten. Die Bewegung ist unsicher:
+ 70% Wahrscheinlichkeit für korrekte Bewegung
+ 10% für seitliche Abweichung (Nord/Süd)
+ 10% für Stehenbleiben

Im 2D-Fall verwenden wir eine **2D-Faltung** mit einem entsprechenden Kernel.

```python                          MotionUpdate2D.py
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import convolve

ROWS, COLS = 8, 10
landmarks = [(0, 2), (0, 7), (2, 0), (2, 9), (4, 1), (4, 6), (6, 3), (6, 8)]

world = np.zeros((ROWS, COLS))
for (r, c) in landmarks:
    world[r, c] = 1

def create_motion_kernel(direction, p_correct=0.7, p_side=0.1, p_stay=0.1):
    """Erstellt 3x3 Bewegungskernel"""
    kernel = np.zeros((3, 3))
    # Kernel-Layout: [[NW, N, NE], [W, Center, E], [SW, S, SE]]

    if direction == 'east':
        kernel[1, 2] = p_correct  # Osten (Hauptrichtung)
        kernel[0, 2] = p_side     # Nordost (Abweichung)
        kernel[2, 2] = p_side     # Südost (Abweichung)
        kernel[1, 1] = p_stay     # Stehen bleiben
    elif direction == 'north':
        kernel[0, 1] = p_correct
        kernel[0, 0] = p_side
        kernel[0, 2] = p_side
        kernel[1, 1] = p_stay

    return kernel / np.sum(kernel)

def motion_update(belief, direction):
    """Predict-Schritt durch 2D-Faltung"""
    kernel = create_motion_kernel(direction)
    return convolve(belief, kernel, mode='wrap')

# Starte mit konzentrierter Belief (Roboter "weiß" ungefähr wo er ist)
belief = np.zeros((ROWS, COLS))
belief[4, 3] = 0.6   # Hauptvermutung
belief[4, 2] = 0.15  # Nebenposition
belief[4, 4] = 0.15
belief[3, 3] = 0.05
belief[5, 3] = 0.05

# Bewegung nach Osten
belief_after = motion_update(belief, 'east')

# Visualisierung
fig, axes = plt.subplots(1, 3, figsize=(15, 4))

ax1 = axes[0]
im1 = ax1.imshow(belief, cmap='Blues')
ax1.set_title('Vor Bewegung')
ax1.plot(3, 4, 'ro', markersize=15)  # Wahre Position
plt.colorbar(im1, ax=ax1)

ax2 = axes[1]
kernel = create_motion_kernel('east')
im2 = ax2.imshow(kernel, cmap='Oranges')
ax2.set_title('Bewegungskernel (Ost)')
for i in range(3):
    for j in range(3):
        ax2.text(j, i, f'{kernel[i,j]:.2f}', ha='center', va='center')
plt.colorbar(im2, ax=ax2)

ax3 = axes[2]
im3 = ax3.imshow(belief_after, cmap='Blues')
ax3.set_title('Nach Bewegung (Ost)')
ax3.plot(4, 4, 'ro', markersize=15)  # Neue wahre Position
plt.colorbar(im3, ax=ax3)

plt.tight_layout()
plt.savefig('foo.png')

print(f"Entropie vorher:  {-np.sum(belief[belief>0] * np.log(belief[belief>0])):.3f}")
print(f"Entropie nachher: {-np.sum(belief_after[belief_after>0] * np.log(belief_after[belief_after>0])):.3f}")
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)

--{{2}}--
Das Ergebnis zeigt einen wichtigen Effekt: Die Entropie, also das Maß für die Unsicherheit, steigt durch die Bewegung an. Die vorher konzentrierte Verteilung "verschmiert" sich. Das ist intuitiv klar: Wenn wir nicht genau wissen, wo wir sind, und uns dann unsicher bewegen, wissen wir danach noch weniger genau, wo wir sind. Deshalb ist der Measurement-Update so wichtig - er ist der einzige Schritt, der Unsicherheit reduziert.

Beachten Sie: Die Unsicherheit (Entropie) nimmt durch die Bewegung zu - die Verteilung "verschmiert".

*******************************************************************************

       {{3-4}}
*******************************************************************************

**Schritt 4: Komplette Simulation - Mehrere Iterationen**

--{{3}}--
Jetzt führen wir alles zusammen und simulieren einen kompletten Durchlauf. Der Roboter startet bei Position Zeile 4, Spalte 1 - direkt neben einer Landmarke. Er fährt dann fünf Schritte nach Osten und passiert dabei eine weitere Landmarke bei Spalte 6. Bei jedem Schritt führen wir erst das Measurement-Update durch, dann das Motion-Update. In der Visualisierung sehen Sie, wie sich die Belief-Verteilung über die Zeit entwickelt. Am Anfang ist alles gleichverteilt. Nach der ersten Messung an der Landmarke konzentriert sich die Wahrscheinlichkeit auf die acht möglichen Positionen. Mit jeder weiteren Bewegung und Messung verfeinert sich die Schätzung, bis der Filter am Ende die korrekte Position identifiziert hat.

Jetzt kombinieren wir alles: Der Roboter bewegt sich durch die Welt und macht wiederholt Messungen.

```python                          BayesFilter2D_Complete.py
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import convolve

# Konfiguration
ROWS, COLS = 8, 10
landmarks = [(0, 2), (0, 7), (2, 0), (2, 9), (4, 1), (4, 6), (6, 3), (6, 8)]
P_HIT, P_MISS = 0.85, 0.1

world = np.zeros((ROWS, COLS))
for (r, c) in landmarks:
    world[r, c] = 1

def measurement_update(belief, detected):
    if detected:
        likelihood = np.where(world == 1, P_HIT, P_MISS)
    else:
        likelihood = np.where(world == 1, 1-P_HIT, 1-P_MISS)
    posterior = belief * likelihood
    return posterior / np.sum(posterior)

def motion_update(belief, direction):
    kernel = np.zeros((3, 3))
    if direction == 'east':
        kernel[1, 2], kernel[0, 2], kernel[2, 2], kernel[1, 1] = 0.7, 0.1, 0.1, 0.1
    elif direction == 'north':
        kernel[0, 1], kernel[0, 0], kernel[0, 2], kernel[1, 1] = 0.7, 0.1, 0.1, 0.1
    elif direction == 'south':
        kernel[2, 1], kernel[2, 0], kernel[2, 2], kernel[1, 1] = 0.7, 0.1, 0.1, 0.1
    kernel = kernel / np.sum(kernel)
    return convolve(belief, kernel, mode='wrap')

# Wahrer Pfad des Roboters
true_path = [(4, 1), (4, 2), (4, 3), (4, 4), (4, 5), (4, 6)]
movements = ['east', 'east', 'east', 'east', 'east']

# Simuliere Sensormessungen (mit Rauschen)
np.random.seed(42)
measurements = []
for (r, c) in true_path:
    has_lm = world[r, c] == 1
    if has_lm:
        measurements.append(np.random.random() < P_HIT)
    else:
        measurements.append(np.random.random() < P_MISS)

# Filter ausführen
belief = np.ones((ROWS, COLS)) / (ROWS * COLS)
beliefs = [belief.copy()]

for i in range(len(true_path)):
    # Measurement Update
    belief = measurement_update(belief, measurements[i])
    beliefs.append(belief.copy())

    # Motion Update (außer beim letzten Schritt)
    if i < len(movements):
        belief = motion_update(belief, movements[i])

# Visualisierung
fig, axes = plt.subplots(2, 3, figsize=(14, 9))
steps_to_show = [0, 1, 2, 3, 4, 5]

for idx, step in enumerate(steps_to_show):
    ax = axes[idx // 3, idx % 3]
    im = ax.imshow(beliefs[step], cmap='Blues', vmin=0, vmax=np.max(beliefs[-1]))

    # Landmarken
    for (r, c) in landmarks:
        ax.plot(c, r, 'g^', markersize=10)

    # Wahre Position
    if step < len(true_path):
        ax.plot(true_path[step][1], true_path[step][0], 'ro', markersize=12,
                markeredgecolor='darkred', markeredgewidth=2)

    # Geschätzte Position (Maximum)
    est = np.unravel_index(np.argmax(beliefs[step]), beliefs[step].shape)
    ax.plot(est[1], est[0], 'bx', markersize=12, markeredgewidth=2)

    meas_str = "LM!" if step > 0 and measurements[step-1] else "---"
    ax.set_title(f'Schritt {step} | Messung: {meas_str}')

plt.suptitle('2D Bayes-Filter: Roboter-Lokalisierung\n(rot=wahr, blau=geschätzt, grün=Landmarken)', fontsize=12)
plt.tight_layout()
plt.savefig('foo.png')

# Finale Auswertung
est_final = np.unravel_index(np.argmax(beliefs[-1]), beliefs[-1].shape)
print(f"Wahre Endposition:     {true_path[-1]}")
print(f"Geschätzte Position:   {est_final}")
print(f"Max. Belief:           {np.max(beliefs[-1]):.4f}")
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)

*******************************************************************************

       {{4-5}}
*******************************************************************************

**Diskussion: Einschränkungen des 2D Bayes-Filters**

--{{4}}--
Das zweidimensionale Beispiel hat gezeigt, dass der diskrete Bayes-Filter funktioniert. Aber es offenbart auch seine fundamentalen Grenzen. Unser kleines 8 mal 10 Gitter benötigt bereits 80 Wahrscheinlichkeitswerte. Stellen Sie sich nun eine realistische Anwendung vor: Eine Fabrikhalle mit 100 mal 100 Metern, diskretisiert in 10-Zentimeter-Zellen. Das ergibt eine Million Zellen! Und dabei haben wir die Orientierung des Roboters noch gar nicht berücksichtigt. Würden wir die Orientierung in 360 Ein-Grad-Schritten einbeziehen, hätten wir 360 Millionen Zellen. Das ist für Echtzeitanwendungen nicht mehr praktikabel.

Das 2D-Beispiel zeigt die Leistungsfähigkeit, aber auch die Grenzen des diskreten Ansatzes:

+ **Speicherbedarf**: Unser 8×10 Grid benötigt 80 Werte. Eine realistische Fabrikhalle (100m × 100m) mit 10cm Auflösung erfordert bereits **1 Million Zellen**!

+ **Rechenaufwand**: Die 2D-Faltung skaliert mit $O(n \cdot k^2)$ wobei $n$ die Zellanzahl und $k$ die Kernelgröße ist.

+ **Diskretisierungsfehler**: Die wahre Position liegt selten exakt auf einem Gitterpunkt.

+ **Multimodalität**: Der Filter kann mehrere Hypothesen gleichzeitig verfolgen (vorteilhaft bei Kidnapped-Robot-Problem), aber dies erhöht auch die Unsicherheit.

--{{4}}--
Allerdings hat der diskrete Ansatz auch einen Vorteil, den wir nicht unterschätzen sollten: Er kann mehrere Hypothesen gleichzeitig verfolgen. Wenn der Roboter entführt und an einem unbekannten Ort abgesetzt wird - das sogenannte Kidnapped-Robot-Problem - kann der diskrete Filter mehrere mögliche Positionen parallel tracken, bis genug Evidenz für eine eindeutige Lokalisierung vorliegt. Diesen Vorteil verlieren wir, wenn wir zu unimodalen Verteilungen wie beim Kalman-Filter übergehen.

![ImageMatching](./images/2DBayes.png)<!-- style="width: 70%;"-->

> **Fazit**: Der diskrete Bayes-Filter eignet sich gut für kleine, strukturierte Umgebungen mit wenigen diskreten Zuständen. Für kontinuierliche Zustandsräume benötigen wir effizientere Repräsentationen → Kalman-Filter (nächste Vorlesung).

*******************************************************************************

## Ausblick: Vom diskreten zum kontinuierlichen Filter

--{{5}}--
Fassen wir zusammen, was wir aus dem diskreten Bayes-Filter gelernt haben, und schauen voraus auf die nächste Vorlesung. Die grundlegende Idee des Predict-Update-Zyklus bleibt erhalten. Was sich ändert, ist die Repräsentation der Unsicherheit. Anstatt für jede Gitterzelle eine Wahrscheinlichkeit zu speichern, nehmen wir an, dass unsere Unsicherheit normalverteilt ist. Eine Normalverteilung wird vollständig durch nur zwei Parameter beschrieben: den Mittelwert und die Varianz. Das reduziert den Speicherbedarf drastisch und ermöglicht geschlossene analytische Lösungen für den Predict- und Update-Schritt. Das Ergebnis ist der Kalman-Filter, den wir in der nächsten Vorlesung kennenlernen werden.

Welche Einschränkungen sehen wir beim diskreten Bayes-Filter?

+ **Speicherbedarf**: Wir müssen für jede Gitterzelle eine Wahrscheinlichkeit speichern. Bei einem 2D-Grid mit 100×100 Zellen sind das bereits 10.000 Werte - bei 3D oder höherer Auflösung explodiert der Speicherbedarf.

+ **Rechenaufwand**: Die Faltungsoperation im Predict-Schritt skaliert mit der Anzahl der Zellen. Für Echtzeit-Anwendungen kann dies problematisch werden.

+ **Diskretisierungsfehler**: Die Auflösung des Gitters begrenzt die Genauigkeit unserer Schätzung. Feinere Gitter erhöhen wiederum Speicher- und Rechenbedarf.

> **Kernfrage**: Können wir die Wahrscheinlichkeitsverteilung kompakter darstellen?

Die Antwort liegt in einer Annahme: Wenn wir davon ausgehen, dass alle Unsicherheiten **normalverteilt** sind, kollabiert die gesamte Verteilung auf nur zwei Parameter - Mittelwert $\mu$ und Varianz $\sigma^2$.

```ascii
    Diskreter Bayes-Filter          Kalman-Filter

    ┌─┬─┬─┬─┬─┬─┬─┬─┬─┬─┐
    │ │▄│█│▄│ │ │ │ │ │ │    ───►      "$\mu, \sigma^2$"
    └─┴─┴─┴─┴─┴─┴─┴─┴─┴─┘
       10 Werte speichern           2 Werte speichern
```

In der **nächsten Vorlesung** werden wir den **Kalman-Filter** kennenlernen, der genau diesen Ansatz verfolgt:

+ Predict-Update-Zyklus bleibt erhalten
+ Zustandsschätzung wird durch Normalverteilung repräsentiert
+ Effiziente Berechnung durch geschlossene Formeln
+ Erweiterung auf nichtlineare Systeme (Extended Kalman Filter)

## Zusammenfassung

+ **Fusionsarten**: Sensordaten können konkurrierend (gleiche Daten → höhere Genauigkeit), komplementär (verschiedene Sichtbereiche → Vollständigkeit) oder kooperativ (kombinierte Daten → neue Information) fusioniert werden.

+ **Fusionsebenen**: Die Fusion kann auf Rohdaten-, Merkmals- oder Entscheidungsebene erfolgen (JDL-Modell).

+ **Diskreter Bayes-Filter**: Iterativer Predict-Update-Zyklus zur Zustandsschätzung basierend auf Wahrscheinlichkeitsverteilungen über diskrete Zustände.

+ **Limitationen**: Hoher Speicher- und Rechenaufwand bei feiner Diskretisierung motiviert den Übergang zu parametrischen Filtern (Kalman-Filter).


