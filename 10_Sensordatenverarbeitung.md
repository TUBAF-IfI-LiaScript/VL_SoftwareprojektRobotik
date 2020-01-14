<!--

author:   Sebastian Zug & Georg Jäger
email:    sebastian.zug@informatik.tu-freiberg.de & Georg.Jaeger@informatik.tu-freiberg.de
version:  1.0.0
language: de
comment:  In dieser Vorlesungen werden die Schichten einer Roboterarchitektur adressiert.
narrator: Deutsch Female

import: https://raw.githubusercontent.com/LiaTemplates/Rextester/master/README.md
import: https://raw.githubusercontent.com/liascript-templates/plantUML/master/README.md

script:   https://cdn.jsdelivr.net/chartist.js/latest/chartist.min.js

link: https://cdn.jsdelivr.net/chartist.js/latest/chartist.min.css

link: https://cdnjs.cloudflare.com/ajax/libs/animate.css/3.7.0/animate.min.css

@eval
<script>
function randn_bm() {
    var u = 0, v = 0;
    while(u === 0) u = Math.random(); //Converting [0,1) to (0,1)
    while(v === 0) v = Math.random();
    let num = Math.sqrt( -2.0 * Math.log( u ) ) * Math.cos( 2.0 * Math.PI * v );
    num = num / 10.0 + 0.5; // Translate to 0 -> 1
    if (num > 1 || num < 0) return randn_bm(); // resample between 0 and 1
    return num;
}

function range(start, end, step = 1) {
  const len = Math.floor((end - start) / step) + 1
  return Array(len).fill().map((_, idx) => start + (idx * step))
}
</script>
@end

-->

# Vorlesung X - Sensordatenverarbeitung

Eine interaktive Version des Kurses finden Sie unter [Link](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/SoftwareprojektRobotik/master/09_Sensordatenverarbeitung.md)

**Zielstellung der heutigen Veranstaltung**

+ Parameterisierung von Meßungenauigkeiten
+ Filterung von Messdaten
+ Schätzung

--------------------------------------------------------------------------------


## Filter

Ein gleitender Mittelwert wird häufig als Allheilmittel für die Filterung von
Messwerten dargestellt. Experimentieren Sie

<link rel="stylesheet" href="//cdn.jsdelivr.net/chartist.js/latest/chartist.min.css">

``` javascript  SlidingWindow.js
// Initialize a Line chart in the container with the ID chart1
function range(start, end, step = 1) {
  const len = Math.floor((end - start) / step) + 1
  return Array(len).fill().map((_, idx) => start + (idx * step))
}

function slidingWindow(randoms, window_size) {
    var result = new Array(randoms.length).fill(null);
    for (var i = 0; i < randoms.length - window_size; i++) {
        var window = randoms.slice(i, i + window_size);
        result[i] = window.reduce(function(p,c,i,a){return p + (c/a.length)},0);
    }
    return result;
}

var xrange = range(0, 4*Math.PI, 4 * Math.PI/100);
var ideal_values = xrange.map(x => Math.sin(x));
var noisy_values = ideal_values.map(x => x + randn_bm() - 0.5);

const window_size = 15;
var mean = slidingWindow(noisy_values, window_size);

new Chartist.Line('#chart1', {
  labels: [1, 2, 3, 4],
  series: [ideal_values, noisy_values, mean]
});
```
<script>@input</script>

<div class="ct-chart ct-golden-section" id="chart1"></div>


<!--
style="width: 70%; max-width: 7200px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii

```





## Aufgabe der Woche
