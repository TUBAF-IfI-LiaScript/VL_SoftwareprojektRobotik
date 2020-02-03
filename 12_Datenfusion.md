<!--

author:   Sebastian Zug & Georg Jäger
email:    sebastian.zug@informatik.tu-freiberg.de & Georg.Jaeger@informatik.tu-freiberg.de
version:  1.0.1
language: de
comment:  In dieser Vorlesungen werden die Schichten einer Roboterarchitektur adressiert.
narrator: Deutsch Female
attribute: thx

import: https://raw.githubusercontent.com/LiaTemplates/Rextester/master/README.md
import: https://raw.githubusercontent.com/liascript-templates/plantUML/master/README.md

script:   https://cdn.jsdelivr.net/chartist.js/latest/chartist.min.js
          https://d3js.org/d3-random.v2.min.js
          https://d3js.org/d3.v4.min.js
          https://cdn.plot.ly/plotly-latest.min.js

link: https://cdnjs.cloudflare.com/ajax/libs/animate.css/3.7.0/animate.min.css
-->

# Vorlesung XII - Datenfusion

Eine interaktive Version des Kurses finden Sie unter [Link](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/SoftwareprojektRobotik/master/12_Datenfusion.md)

**Zielstellung der heutigen Veranstaltung**

+ Vermittlung eines Grundverständnisses für die Datenfusion
+ Einführung eines Bayes basierten Schätzers als diskreten Filter
+ Ausblick auf die Praktischen Themenstellungen des folgenden Semesters

--------------------------------------------------------------------------------

## Auswertung der Studierenden Befragung

1. _"Viel Wissen wird aus SWE vorausgesetzt. Wenn man einmal aus dem Thema raus ist, ist es schwer wieder rein zukommen"_

2. _"Der Übergang von SWE zum Projekt ist ein riesiger Sprung. Innerhalb weniger Wochen C++ perfekt zu lernen und dann in die Spezialisierung von ROS einzutauchen ist extrem viel Lernstoff in einer sehr geringen Zeit."_

3. Wie viele Stunden haben Sie im Schnitt pro Woche investiert?  

     2 x 1 bis 2h, 1 x 2 bis 3h, 1 x größer 4h

4. Komplexe Sachverhalte werden inhaltlich verständlich dargestellt.

     teils-teils

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
Fusion         | Konkurierend  |    | Komplementär  |     | Kooperativ    |
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

+ Eine __komplementäre Fusion__ hat das Ziel, die Vollständigkeit der Daten zu erhöhen. Unabhängige Sensoren betrachten hierfür unterschiedliche Sichtbereiche und Phänomene oder messen zu unterschiedlichen Zeiten.

+ Bei der __konkurrierenden Fusion__ erfassen Sensoren gleichzeitig denselben Sichtbereich und liefern Daten gleicher Art. Die (oft gewichtete) Verknüpfung solcher, "konkurrierender" Daten kann die Genauigkeit des Gesamtsystems erhöhen.

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

![ImageMatching](./img/12_Datenfusion/JDLmodell.png)<!-- style="width: 70%;"-->

_Sensor and Data Fusion: Concepts and Application, second ed., vol. TT14SPIE Press (1999)_

1. Vorverarbeitung : zeitliche und räumliche Registrierung der Daten sowie Vorverarbeitungsmaßnahmen auf Signal- oder Pixelebene.
2. Objekterkennung und Extraktion Schätzung und Vorhersage von kontinuierlichen oder diskreten Objektmerkmalen.
3. Situationsanalyse: alle detektierten Objekte werden in einen größeren Kontext gebracht und Objektbeziehungen analysiert werden.
4. Bedrohungsanalyse: Situationsabhängig werden, im Sinne einer Risikominimierung, unterschiedliche Handlungsoptionen evaluiert.
5. Prozessoptimierung: Verwaltung und Adaption der Ressourcen

*******************************************************************************

## Einführung in den diskreten Bayes Filter

Das Beispiel basiert auf der hervorrangenden Einführung in die Mathematik und Anwendung von Fusionsansätzen des

https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/02-Discrete-Bayes.ipynb

Das Anwendungsbeispiel zielt auf den Schwimmroboter, der sich auf der Freiberger Mulde bewegt, lässt sich aber analog auf beliebige andere 1D, 2D oder 3D Szenarien übertragen.

Anstatt eine kontinuierliche Messung zu realisieren bilden wir unsere Position auf diskrete Streckenabschnitte ab. Im Bild oben werden diese mit 0-9 indiziert.

<!--
style="width: 50%; min-width: 380px; max-width: 720px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii

  Streckensegmente der Freiberger Mulde

| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
======O===SB======O===O===B===S=======S== -->
                                          Strömung

```   
_Abbildung des Streckenmodels (O = Orange Warnschilder am Ufer, S = Starke Strömung, B = Brücken)_

Welche Anfangskenntnis zur Position haben wir zunächst? Die Aufenthaltswahrscheinlichkeit ist für alle 10 Streckensegmente gleich und entsprechend $p_i=0.1$ für alle $0 \leq i < 10$

```python                          generateBelief.py
import numpy as np
belief = np.array([1./10]*10)
print(belief)
```
```js -Visualization
var line = data.Result.slice(1, data.Result.length-2);
line = line.replace( /\s\s+/g, ' ' );
var outcome = line.split(' ').map(function(item) {
    return parseFloat(item);
});

var plot = [
  {
    x: d3.range(0, 10),
    y: outcome,
    type: 'bar',
    name: 'Potential positions',
  }
];

var layout = {
    height : 300,
    width :  650,
    yaxis: {
      range: [0, 1],
      title: {
        text: 'Probablity p',
      },
    },
    xaxis: {
      title: {
        text: 'Segments i',
      },
    },
    margin: { l: 60, r: 10, b: 35, t: 10, pad: 4},
    showlegend: true,
    legend: { x: 1, xanchor: 'right', y: 1},
    tracetoggle: false
};

Plotly.newPlot('Diagram1', plot, layout);
console.log("Aus Maus")
```@Rextester._eval_(@uid, @Python,`@0`,`@1`,` `,`@input(1)`)

<div id="Diagram1"></div>

Es wird deutlich, dass wir aktuell noch kein Wissen um die Position des Bootes haben. Man spricht vom "apriori-"Wissen.

### Sensoren

Unser Roboter ist mit verschiedenen Sensorsystemen ausgestattet.

1. Im Frontbereich des Roboters befindet sich ein Kamerasystem, dass aus Bilddaten features zu erkennen versucht. Dabei konzentrierte man sich auf orange Warnschilder und Brücken über die Mulde.

![ImageMatching](./img/12_Datenfusion/opencv_matching.jpg)<!-- style="width: 50%;"-->

*Examplarische Darstellung eines Image Matching Ansatzes auf der Basis einer SIFT Methode* [OpenCV Tutorial](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_matcher/py_matcher.html)

2. Ein Beschleunigungssensor bestimmt die Bewegungen des Bootes. Damit lassen sich zum Beispiel starke Strömungen gut erkennen.

3. An einige Stellen können mit dem GNSS-System Messdaten generiert werden. Die umgebenden Höhenzüge behindern dies.

Alle drei Messmethoden sind mit Fehlern überlagert. Welche können das sein?

### Bildsensor

Nun nutzen wir den ersten Sensormodus, die Erkennung der orangen Zeichen. Wir gehen davon aus, dass der Sensor nur in seltenen Fällen eine falsche Klassifikation (Schild/kein Schild) vornimmt.

<!--
style="width: 100%; min-width: 380px; max-width: 720px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii

                   Streckensegmente der Freiberger Mulde

                 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
                 ======O===SB======O===O===B===S=======S== -->
                                                           Strömung            

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
                           ja    | 0.8  | 0.3  |                               
           Schild erkannt        +------+------+
                         nein    | 0.2  | 0.7  |
                                 +------+------+
```

_Vierfeldertafel unseres "Schilderkenners"_

Wie wahrscheinlich ist es also, dass wir uns tatsächlich an einem orangen Schild
befinden, wenn wir eine entsprechende Messung vorliegen haben?

$$p(x|z) = \frac{0.8}{0.8 \cdot 0.3} = 0.72$$

Wie wahrscheinlich ist eine Messung eines orangen Schildes, wenn wir gar keines erreicht haben?

$$p(x|z) = \frac{0.3}{0.8 \cdot 0.3} = 0.27$$

Wie können wir diese Sensorcharakteristik nun für unser Roboterbeispiel verwenden? Unsere Klassifikation wird durch die verschiedenen Möglichkeiten, an denen sich der Roboter aufhalten kann, "verwässert". Wenn wir nur ein Segment mit einem Schild hätten müssten wir diesem eine Aufenthaltswahrscheinlichkeit von $p=0.72$ zuordnen. Da es aber mehrere Möglichkeiten gibt splitet sich diese auf.

```python                          generateMeasurements.py
import numpy as np

markers = np.array([0, 1., 0, 0, 1., 1., 0, 0, 0, 0])
truePositive = 0.72
belief = markers / sum(markers)*truePositive
belief[markers == 0] = (1-truePositive)/np.count_nonzero(markers==0)
print(belief)
```
```js -Visualization
var line = data.Result.slice(1, data.Result.length-2);
line = line.replace( /\s\s+/g, ' ' );
var outcome = line.split(' ').map(function(item) {
    return parseFloat(item);
});

var plot = [
  {
    x: d3.range(0, 10),
    y: outcome,
    type: 'bar',
    name: 'Orange signs measurements',
  }
];

var layout = {
    height : 300,
    width :  650,
    yaxis: {
      range: [0, 1],
      title: {
        text: 'Probablity p',
      },
    },
    xaxis: {
      title: {
        text: 'Segments i',
      },
    },
    margin: { l: 60, r: 10, b: 35, t: 10, pad: 4},
    showlegend: true,
    legend: { x: 1, xanchor: 'right', y: 1},
    tracetoggle: false
};

Plotly.newPlot('Diagram2', plot, layout);
console.log("Aus Maus")
```@Rextester._eval_(@uid, @Python,`@0`,`@1`,` `,`@input(1)`)

<div id="Diagram2"></div>

### Abbildung auf apriori Wissen

Was bedeutet dies aber im Hinblick auf die Positionsbestimmung unter Berücksichtigung des apriori-Wissens?

Wir haben zwei Vektoren mit Wahrscheinlichkeiten bezüglich unseres Aufenthaltes. Die Kombination draus ist eine einfaches Produkt gefolgt von einer Normalisierung.

$$posteriori = \frac{likelihood \cdot prior}{normalization}$$


```python                          generateBelief.py
import numpy as np
apriori = np.array([1./10]*10)
markers = np.array([0, 1., 0, 0, 1., 1., 0, 0, 0, 0])
truePositive = 0.72
belief = markers / sum(markers)*truePositive
belief[markers == 0] = (1-truePositive)/np.count_nonzero(markers==0)
posteriori = (apriori * belief) / sum(apriori*belief)
print(posteriori)
```
```js -Visualization
var line = data.Result.slice(1, data.Result.length-2);
line = line.replace( /\s\s+/g, ' ' );
var outcome = line.split(' ').map(function(item) {
    return parseFloat(item);
});

var plot = [
  {
    x: d3.range(0, 10),
    y: outcome,
    type: 'bar',
    name: 'Posteriori probabilities',
  }
];

var layout = {
    height : 300,
    width :  650,
    yaxis: {
      range: [0, 1],
      title: {
        text: 'Probablity p',
      },
    },
    xaxis: {
      title: {
        text: 'Segments i',
      },
    },
    margin: { l: 60, r: 10, b: 35, t: 10, pad: 4},
    showlegend: true,
    legend: { x: 1, xanchor: 'right', y: 1},
    tracetoggle: false
};

Plotly.newPlot('Diagram2', plot, layout);
console.log("Aus Maus")
```@Rextester._eval_(@uid, @Python,`@0`,`@1`,` `,`@input(1)`)

<div id="Diagram2"></div>

Wie erwartet haben die Segmente mit Schildern eine deutlich höhere Wahrscheinlichkeit von $p=0.24$ als die anderen Bereiche. Welche Veränderung
erwarten Sie, wenn wir die Qualität der Sensormessungen erhöhen?

Wie können wir unsere Positionsschätzung verbessern:

1. Verbesserung der Qualität der Sensoren
2. Einbettung weiterer Sensoren
3. Wiederholung der Messungen (sofern wir von statistisch unabhängigen Messungen ausgehen)

Lassen Sie uns die Messung einige Male wiederholen und diese Erkenntnis einfließen.

```python                          constantPosition.py
import numpy as np
apriori = np.array([1./10]*10)
markers = np.array([0, 1., 0, 0, 1., 1., 0, 0, 0, 0])
truePositive = 0.72
belief = markers / sum(markers)*truePositive
belief[markers == 0] = (1-truePositive)/np.count_nonzero(markers==0)
p_1 = []; p_1.append(apriori[1])
posteriori = apriori
for i in range(1, 10):
  posteriori = (posteriori * belief) / sum(posteriori*belief)
  p_1.append(posteriori[1])
print(p_1)
```
```js -Visualization
var line = data.Result.slice(1, data.Result.length-2);
line = line.replace( /\s\s+/g, ' ' );
var outcome = line.split(' ').map(function(item) {
    return parseFloat(item);
});

var plot = [
  {
    x: d3.range(0, 10),
    y: outcome,
    type: 'bar',
    name: 'Probability of segment [1]'
  }
];

var layout = {
    height : 300,
    width :  650,
    yaxis: {
      range: [0, 1],
      title: {
        text: 'Probablity p',
      },
    },
    xaxis: {
      title: {
        text: 'Number of repetitions n',
      },
    },
    margin: { l: 60, r: 10, b: 35, t: 10, pad: 4},
    showlegend: true,
    legend: { x: 1, xanchor: 'right', y: 1},
    tracetoggle: false
};

Plotly.newPlot('Diagram3', plot, layout);
console.log("Aus Maus")
```@Rextester._eval_(@uid, @Python,`@0`,`@1`,` `,`@input(1)`)

<div id="Diagram3"></div>

Unsere Positionsschätzung nähert sich der belief-Verteilung unsere Messung an. Der Einfluß des Anfangswissens geht zurück.

### Bewegung

Die über der Zeit gestiegene Qualität der Positionsschätzung lässt sich bisher nur realisieren, wenn keine Bewegung erfolgt. Wie aber kann diese abgebildet werden?

Nehmen wir wiederum eine Aufenthaltwahrscheinlichkeit über unseren Segmenten an, die durch eine Bewegung $u$ verändert wird. $u$ ist dabei die Kontrollfunktion, mit der wir unserer Umgebung/Roboter manipulieren Der einfachheit halber berücksichtigen wir zunächst eine ungestörtet Abbildung. Für $u=v=1$ verschiebt sich unser Roboter um eine Einheit.

```python                          movements.py
import numpy as np

def perfect_predict(belief, u):
    n = len(belief)
    result = np.zeros(n)
    for i in range(n):
        result[i] = belief[(i-u) % n]
    return result

belief = np.array([0.05, .35, .1, .2, .3, 0, 0, 0, 0, 0, 0])
print(belief)
print(perfect_predict(belief, 1))
```
```js -Visualization
var lines = data.Result.split('\n');
var line_0 = lines[0].slice(1, lines[0].length-1).replace( /\s\s+/g, ' ' );
var line_1 = lines[1].slice(1, lines[1].length-1).replace( /\s\s+/g, ' ' );

var outcome_0 = line_0.split(' ').map(function(item) {
    return parseFloat(item);
});

var outcome_1 = line_1.split(' ').map(function(item) {
    return parseFloat(item);
});

var plot = [
  {
    x: d3.range(0, 10),
    y: outcome_0,
    type: 'bar',
    name: 'Before prediction'
  },
  {
    x: d3.range(0, 10),
    y: outcome_1,
    type: 'bar',
    name: 'After prediction'
  },  
];

var layout = {
    barmode: 'group',
    height : 300,
    width :  650,
    yaxis: {
      range: [0, 1],
      title: {
        text: 'Probablity p',
      },
    },
    xaxis: {
      title: {
        text: 'Segment i',
      },
    },
    margin: { l: 60, r: 10, b: 35, t: 10, pad: 4},
    showlegend: true,
    legend: { x: 1, xanchor: 'right', y: 1},
    tracetoggle: false
};

Plotly.newPlot('Diagram4', plot, layout);
console.log("Aus Maus")
```@Rextester._eval_(@uid, @Python,`@0`,`@1`,` `,`@input(1)`)

<div id="Diagram4"></div>

Welche Probleme sehen Sie?

Allerdings ist es völlig unrealistisch, dass sich unser System perfekt verhält. Strömungen und Dynamikeinflüsse generieren situative Verstärkungen oder Abschwächungen der Geschwindigkeit.

Zunächst berücksichtigen nur die beiden benachbarten Felder und ein perfektes apriori Wissen. Durch die Strömung bedingt verlässt unser Roboter die Position 2 trotz einer Beschleunigung stromab nicht, möglicherweise bewegt er sich aber auch zwei Segmente weiter.

```python                          uncertainmovements.py
import numpy as np

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
print(belief)
print( predict_move(belief, 1, .1, .8, .1))
```
```js -Visualization
var lines = data.Result.split('\n');
var line_0 = lines[0].slice(1, lines[0].length-1).replace( /\s\s+/g, ' ' );
var line_1 = lines[1].slice(1, lines[1].length-1).replace( /\s\s+/g, ' ' );

var outcome_0 = line_0.split(' ').map(function(item) {
    return parseFloat(item);
});

var outcome_1 = line_1.split(' ').map(function(item) {
    return parseFloat(item);
});

var plot = [
  {
    x: d3.range(0, 10),
    y: outcome_0,
    type: 'bar',
    name: 'Before prediction'
  },
  {
    x: d3.range(0, 10),
    y: outcome_1,
    type: 'bar',
    name: 'After prediction'
  },  
];

var layout = {
    barmode: 'group',
    height : 300,
    width :  650,
    yaxis: {
      range: [0, 1],
      title: {
        text: 'Probablity p',
      },
    },
    xaxis: {
      title: {
        text: 'Segment i',
      },
    },
    margin: { l: 60, r: 10, b: 35, t: 10, pad: 4},
    showlegend: true,
    legend: { x: 1, xanchor: 'right', y: 1},
    tracetoggle: false
};

Plotly.newPlot('Diagram5', plot, layout);
console.log("Aus Maus")
```@Rextester._eval_(@uid, @Python,`@0`,`@1`,` `,`@input(1)`)

<div id="Diagram5"></div>

Was aber geschieht, wenn wir von einem unsicheren priori Wissen ausgehen?

```
//        0   1    2   3   4   5   6   7   8   9
belief = [0., 0., .4, .6,  0., 0., 0., 0., 0., 0.]
```

Wie groß ist die Wahrscheinlichkeit, dass wir Segment x erreichen? Dabei berücksichtigen wir nun die verschiedenen Startpunkte, in dem wir die Abbildungsfunktion von $u$ auf alle Kombinationen anwenden.

| Segment | Rechnung | Ergebnis |
|---------|----------|----------|
|   0     |          | nicht erreichbar |
|   1     |          | nicht erreichbar |
|   2     | $0.4 \cdot 0.1$         |  $0.04$ |
|   3     | $0.4 \cdot 0.8 + 0.6 \cdot 0.1$  | $0.38$  |  
|   4     | $0.4 \cdot 0.1 + 0.6 \cdot 0.8$  | $0.52$  |  
|   5     | $0.6 \cdot 0.1$  | $0.06$  |  
|   6     |          | nicht erreichbar |
|   7     |   | ... |

Grafisch dargestellt ergibt sich damit folgendes Bild:

<!--
style="width: 100%; min-width: 380px; max-width: 720px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii
               |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  |

      priori      0     0    0.4   0.6    0     0     0     0     0     0      
    [0.1   0.8   0.1]  -->  

                [0.1   0.8   0.1]  
                             0.04                

                      [0.1   0.8   0.1]  
                                   0.38
```   
_Abbildung des Streckenmodels (O = Orange Warnschilder am Ufer, S = Starke Strömung, B = Brücken)_

Damit bilden wir eine diskrete Faltungsoperation über unserer Aufenthaltswahrscheinlichkeit ab. Die Defintion der Faltung ergibt sich zu

$$(f * g)(n) = \sum_{k \in D} f(k) g(n - k)$$

Wenn wir die Rechung also verallgemeinern können wir auf eine bestehende Implementierung zurückgreifen. Die `scipy` Bibliothek hält eine Funktion `convolve` bereit [Link](https://docs.scipy.org/doc/scipy/reference/generated/scipy.ndimage.convolve.html)

```python                          uncertainmovements.py
import numpy as np
from scipy import ndimage

belief = [.05, .5, .05, .05, .05, .05, .05, .05, .05, .05]
print(belief)
kernel = [.1, 0.8, 0.1]
prior = ndimage.convolve(np.roll(belief, len(kernel) / 2), kernel, mode='wrap')
print(prior)
#belief = prior   # Multiple movements
#prior = ndimage.convolve(np.roll(belief, len(kernel) / 2), kernel, mode='wrap')
#print(prior)
```
```js -Visualization
var lines = data.Result.split('\n');
var line_0 = lines[0].slice(1, lines[0].length-1).replace( /\s\s+/g, ' ' );
var line_1 = lines[1].slice(1, lines[1].length-1).replace( /\s\s+/g, ' ' );
var line_2 = lines[2].slice(1, lines[2].length-1).replace( /\s\s+/g, ' ' );

var outcome_0 = line_0.split(' ').map(function(item) {
    return parseFloat(item);
});

var outcome_1 = line_1.split(' ').map(function(item) {
    return parseFloat(item);
});

var outcome_2 = line_2.split(' ').map(function(item) {
    return parseFloat(item);
});

var plot = [
  {
    x: d3.range(0, 10),
    y: outcome_0,
    type: 'bar',
    name: 'Before movement'
  },
  {
    x: d3.range(0, 10),
    y: outcome_1,
    type: 'bar',
    name: 'After movement'
  },  
  {
    x: d3.range(0, 10),
    y: outcome_2,
    type: 'bar',
    name: 'After second movement'
  },
];

var layout = {
    barmode: 'group',
    height : 300,
    width :  650,
    yaxis: {
      range: [0, 1],
      title: {
        text: 'Probablity p',
      },
    },
    xaxis: {
      title: {
        text: 'Segment i',
      },
    },
    margin: { l: 60, r: 10, b: 35, t: 10, pad: 4},
    showlegend: true,
    legend: { x: 1, xanchor: 'right', y: 1},
    tracetoggle: false
};

Plotly.newPlot('Diagram6', plot, layout);
console.log("Aus Maus")
```@Rextester._eval_(@uid, @Python,`@0`,`@1`,` `,`@input(1)`)

<div id="Diagram6"></div>

Mit jedem Prädiktionsschritt fächert die Breite der Unsicherheit entsprechend auf.

### Und jetzt alles zusammen

Fassen wir nun beide Aspekte, die Vorhersage des Systemverhaltens und die Korrektur anhand der Messdaten zusammen. Aus dieser Konstallation wird deutlich, dass wir einen iterativen Prozess realisieren, in dessen Ablauf eine Zustandsvariable, hier unser Positionsindex "verfolgt" wird.

| Zustand         | Bedeutung |
|-----------------|-----------|
| Initialisierung | Definition einer Anfangsschätzung |
| Vorhersage      | Auf der Grundlage des unsicheren Systemverhaltens den Zustand für den nächsten Zeitschritt vorhersagen |
| Aktualisierung  | Eintreffen einer Messung, Abbildung auf die Wahrscheinlichkeit eines Systemzustandes |


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

Das folgende Codefragment bildet zwei Iterationen für unser Beispiel ab. Im ersten Durchlauf ändert die Prediktionsphase den Intertialen Wissenstand nicht. Die Faltung des Kernels ändert die Aufenthaltwahrscheinlichkeit nicht. Eine  Präzisierung erfährt diese mit der ersten Messung durch den Schildersensor.

```python                          BayesFilter.py
import numpy as np
from scipy import ndimage

priori = np.array([1./10]*10)
kernel = [.1, 0.8, 0.1]
markers = np.array([0, 1., 0, 0, 1., 1., 0, 0, 0, 0])
truePositive = 0.72
belief = markers / sum(markers)*truePositive
belief[markers == 0] = (1-truePositive)/np.count_nonzero(markers==0)
# FIRST LOOP - prediction
posteriori = ndimage.convolve(np.roll(priori, len(kernel) / 2), kernel, mode='wrap')
print(posteriori)
# FIRST LOOP - update
priori = posteriori
posteriori = priori * belief / sum(priori * belief )
print(posteriori)
# SECOND LOOP - prediction
priori = posteriori
posteriori = ndimage.convolve(np.roll(priori, len(kernel) / 2), kernel, mode='wrap')
print(posteriori)
# SECOND LOOP - update
posteriori = posteriori * belief / sum(posteriori * belief )
print(posteriori)
```
```js -Visualization
var lines = data.Result.split('\n');
var line_0 = lines[0].slice(1, lines[0].length-1).replace( /\s\s+/g, ' ' );
var line_1 = lines[1].slice(1, lines[1].length-1).replace( /\s\s+/g, ' ' );
var line_2 = lines[2].slice(1, lines[2].length-1).replace( /\s\s+/g, ' ' );
var line_3 = lines[3].slice(1, lines[3].length-1).replace( /\s\s+/g, ' ' );

var outcome_0 = line_0.split(' ').map(function(item) {
    return parseFloat(item);
});

var outcome_1 = line_1.split(' ').map(function(item) {
    return parseFloat(item);
});

var outcome_2 = line_2.split(' ').map(function(item) {
    return parseFloat(item);
});

var outcome_3 = line_3.split(' ').map(function(item) {
    return parseFloat(item);
});

var plot1 = [
  {
    x: d3.range(0, 10),
    y: outcome_0,
    type: 'bar',
    name: 'Apriori Knowledge'
  },
  {
    x: d3.range(0, 10),
    y: outcome_1,
    type: 'bar',
    name: 'After measurement'
  },  
];

var plot2 = [
  {
    x: d3.range(0, 10),
    y: outcome_2,
    type: 'bar',
    name: 'After prediction'
  },
  {
    x: d3.range(0, 10),
    y: outcome_3,
    type: 'bar',
    name: 'After measurement'
  },  
];

var layout = {
    barmode: 'group',
    height : 300,
    width :  650,
    yaxis: {
      range: [0, 1],
      title: {
        text: 'Probablity p',
      },
    },
    xaxis: {
      title: {
        text: 'Segment i',
      },
    },
    margin: { l: 60, r: 10, b: 35, t: 10, pad: 4},
    showlegend: true,
    legend: { x: 1, xanchor: 'right', y: 1},
    tracetoggle: false
};

Plotly.newPlot('Diagram7', plot1, layout);
Plotly.newPlot('Diagram8', plot2, layout);
console.log("Aus Maus")
```@Rextester._eval_(@uid, @Python,`@0`,`@1`,` `,`@input(1)`)

<div id="Diagram7"></div>
<div id="Diagram8"></div>

Welche Einschränkungen sehen Sie in dem Beispiel?

+ Wir bilden nur eine Zustandsvariable ab. Bereits die Umsetzung eines 2D oder 3D Beispiels würde eine erhebliche Anpassung notwendig machen. Damit würde dann aber auch die Komplexität und die Berechnungsdauer entsprechend ansteigen.

![ImageMatching](./img/12_Datenfusion/2DBayes.png)<!-- style="width: 70%;"-->

+ Diese Überlegung ist in starkem Maße mit der Frage nach der Auflösung unserer Diskretisierung verbunden. Ein 100x100m große Fabrikhallen, die mit einem 10cm Grid überzogen werden soll, bedeutet, dass wir jeweils 1 Million Kacheln evaluieren müssen.

+ Die Abbildung der Sensorunsicherheit ist hier stark vereinfacht. Das Fehlermodell allein auf die Klassifikationsgüte abzubilden genügt in der Regel nicht. Die Ausgabe unseres Kamerasystems wird als statt einer konstanten Abbildungsfunktion der Messungen auf die Zustände eher ein variables Qualitätsattribut realisieren.

+ Die Modalität des Sensors wurde so gewählt, dass dessen Daten einfach zu integrieren sind. Wie würden Sie die Informationen des Beschleunigungssensors berücksichtigen?

## Ausblick

      {{0-1}}
*********************************************************************************
> Merke: Datenfusion generiert einen erheblichen Aufwand und erfordert Annahmen zur Umgebung des Systems. Gleichzeitig ist sie kein Garant für ein funktionsfähiges System!

![ImageMatching](./img/12_Datenfusion/Nahin.png)<!-- style="width: 70%;"-->

_Nahin, John L., Can Two Plus Two Equal Five? 1980_

*********************************************************************************
          {{1}}
<font size="4"> Wie geht es im nächsten Semester weiter?</font>

          {{2}}
<font size="4"> Danke für das Interesse!</font>

## Aufgaben

+ Variieren Sie die Parameter der Beispiele der Implementierung und evaluieren Sie den Einfluss unterschiedlicher Bewegungsmodelle
+ Integrieren Sie eine Fusion mit den übrigen angedeuteten Sensoren, die auf dem Roboter verbaut sind.
