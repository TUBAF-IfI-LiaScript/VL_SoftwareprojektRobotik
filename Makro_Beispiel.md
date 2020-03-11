<!--

author:   Sebastian Zug
email:    sebastian.zug@informatik.tu-freiberg.de
version:  1.0.2
language: de
comment:  In dieser Vorlesungen werden die Schichten einer Roboterarchitektur adressiert.
narrator: Deutsch Female

import: https://raw.githubusercontent.com/LiaTemplates/Rextester/master/README.md

attribute: Danke an Andre Dietrich für seinen Kurs "Einführung Regelungstechnik" aus dem Teile übernommen wurden.

script:   https://cdn.jsdelivr.net/chartist.js/latest/chartist.min.js
          https://d3js.org/d3-random.v2.min.js
          https://d3js.org/d3.v4.min.js
          https://cdn.plot.ly/plotly-latest.min.js

link: https://cdnjs.cloudflare.com/ajax/libs/animate.css/3.7.0/animate.min.css

@eval
@Rextester._eval_(@uid, @Python, , , ,```
    var lines = data.Result.replace(/\n/g, ' ').replace(/\s{2,}/g, ' ').match(/(?<=\[).+?(?=\])/g);

    var outcome = [];
    for (var i=0; i<lines.length; i++){
      outcome[i] = lines[i].split(' ').map(function(item) {
          return parseFloat(item);
      });  
    }
    @input(1);
    Plotly.newPlot(span_id, traces, layout);
    console.log("Aus Maus");
```)
@end
-->

# Example for Macro usage in LiaScript

Let us assume that we intend to explain a PT1 behavior and its simulation in a lecture, based on LiaScript. The implementation has to be done in python while the visualization needs to be realized in js. Hence, we need a parser that extracts the actual values and provides the diagrams. The macro `@eval` calls another macro definition `@Rextester._eval_` that coordinates these activities.

<!--
style="width: 60%; min-width: 420px; max-width: 720px;"
-->
```ascii
  +-------------------------------------------------------+
  |  +--------------------------------------+             |
  |  | Rextester Python Implementation      |   @input(0) |
  |  +--------------------------------------+             |
  |                  | print(u, v)                        |
  |                  v                                    |
  |  +--------------------------------------+             |
  |  | JavaScript Snippet for visualization |   @input(1) |
  |  +--------------------------------------+             |
  +-------------------------------------------------------+
  @Rextester._eval_(@uid, @Python, , , , code)
```   

The macro `@eval` is contained in the header. `_input(0)_` is called automatically. The first ten lines of the macro code include the parsing and extraction process based on `_input(0)_` output. The next step executes the visualization script by its reference `_input(1)_`. Extract the code by clicking on the corresponding blue bar in the code section. The specific implementation is hidden by default. The last step initiates the actual drawing process.

@@eval

The example visualizes a PT1-element. Test different parameter settings for an evaluation of their effect!

```python                          System.py
import numpy as np

# Parameters of the system
T = 0.01           # time constant of the system
deltaT = 0.001     # sample frequency
K = 1              # Final value

# Input values
u = np.zeros(50); u[10:20]=1; u[30:-1]=1
T_star = 1 / ((T / deltaT) + 1)

# Simulation of the system
y = np.zeros(len(u))
for index, entry in enumerate(u):
    if index>0:
        y[index] = T_star*(K*u[index]-y[index-1])+y[index-1]

print(u)
print(y)
```
```js -Visualization
var traces = [
  {
    x: d3.range(0, 100),
    y: outcome[0],
    mode: 'lines',
    line: {shape: 'vh'},
    type: 'scatter',
    name: 'Activation u',
  },
  {
    x: d3.range(0, 100),
    y: outcome[1],
    type: 'scatter',
    name: 'System response y',
  }
];

var layout = {
    height : 300,
    width :  650,
    yaxis: {
      range: [0, 1],
      title: {
        text: 'System value',
      },
    },
    xaxis: {
      title: {
        text: 'Samples',
      },
    },
    margin: { l: 60, r: 10, b: 35, t: 10, pad: 4},
    showlegend: true,
    legend: { x: 1, xanchor: 'right', y: 0},
    tracetoggle: false
};
```
@eval
