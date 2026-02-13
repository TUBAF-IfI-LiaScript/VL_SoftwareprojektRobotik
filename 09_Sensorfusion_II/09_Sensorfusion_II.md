<!--

author:   Sebastian Zug & Georg Jäger
email:    sebastian.zug@informatik.tu-freiberg.de & Georg.Jaeger@informatik.tu-freiberg.de
version:  1.0.3
language: de
comment:  In dieser Vorlesung wird der Kalman-Filter als Methode zur Sensordatenfusion eingeführt.
narrator: Deutsch Female
attribute: thx

import:   https://github.com/liascript/CodeRunner
          https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/config.md

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/09_Sensorfusion_II/09_Sensorfusion_II.md)

# Sensordatenfusion II: Kalman-Filter

<!-- data-type="none" -->
| Parameter            | Kursinformationen                                                                                             |
| -------------------- | ------------------------------------------------------------------------------------------------------------- |
| **Veranstaltung:**   | @config.lecture                                                                                               |
| **Semester**         | @config.semester                                                                                              |
| **Hochschule:**      | `Technische Universität Freiberg`                                                                             |
| **Inhalte:**         | `Kalman-Filter, Extended Kalman Filter, robot_localization`                                                   |
| **Link auf GitHub:** | https://github.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/blob/master/09_Sensorfusion_II/09_Sensorfusion_II.md |
| **Autoren**          | @author                                                                                                       |

![](https://media.giphy.com/media/3o7btPCcdNniyf0ArS/giphy.gif)

--------------------------------------------------------------------------------

**Zielstellung der heutigen Veranstaltung**

+ Verstehen der mathematischen Grundlagen des Kalman-Filters
+ Implementierung eines 1D Kalman-Filters
+ Erweiterung auf mehrdimensionale Systeme
+ Einführung des Extended Kalman Filters für nichtlineare Systeme
+ Kennenlernen des `robot_localization` Pakets in ROS 2

--------------------------------------------------------------------------------

## Einschub - Quality of Service (QoS) für Sensordaten

--{{0}}--
Ein wichtiger Aspekt bei der Sensorfusion in ROS 2 ist die Quality of Service Konfiguration. QoS bestimmt, wie zuverlässig und mit welcher Latenz Sensordaten übertragen werden. Für die Sensorfusion ist das entscheidend: Wollen wir lieber alle Daten garantiert erhalten, auch wenn das Verzögerungen bedeutet? Oder ist uns eine niedrige Latenz wichtiger, selbst wenn mal ein Paket verloren geht?

ROS 2 bietet eine Vielzahl von Quality of Service (QoS)-Richtlinien, mit denen Sie die Kommunikation zwischen Knoten und die Datenhaltung optimieren können. Eine Reihe von QoS "Richtlinien" kombinieren sich zu einem QoS "Profil".

**Wichtige QoS-Parameter:**

+ *Durability* ... legt fest, ob und wie lange Daten, die bereits ausgetauscht worden sind, "am Leben bleiben". `volatile` bedeutet, dass dahingehend kein Aufwand investiert wird, `transient` oder `persistent` gehen darüber hinaus.
+ *Reliability* ... definiert, ob alle geschriebenen Datensätze bei allen Readern angekommen sein müssen. `reliable` stellt Daten zuverlässig zu (mit Wiederholung bei Verlust), `best effort` liefert schnellstmöglich ohne Garantie.
+ *History* ... definiert, wie viele der letzten Daten gespeichert werden. `Keep last` speichert n Samples (definiert durch _Depth_), `Keep all` speichert alle Samples.
+ *Depth* ... erfasst die Größe der Queue für die History, wenn `Keep last` gewählt wurde.

**Vordefinierte QoS-Profile in ROS 2:**

| Profil                 | Durability | Reliability | History   | Depth  |
| ---------------------- | ---------- | ----------- | --------- | ------ |
| Publisher & Subscriber | volatile   | reliable    | keep last | -      |
| Services               | volatile   | reliable    | keep last | -      |
| Sensor data            | volatile   | best effort | keep last | small  |
| Parameters             | volatile   | reliable    | keep last | larger |
| Default                | volatile   | reliable    | keep last | small  |

--{{1}}--
Beachten Sie, dass Sensordaten standardmäßig mit "best effort" konfiguriert sind! Das bedeutet schnelle Übertragung, aber keine Garantie. Für die Sensorfusion mit robot_localization ist das meist in Ordnung, da der Filter mit fehlenden Messungen umgehen kann.

**Kompatibilität zwischen Publisher und Subscriber:**

| Publisher   | Subscriber  | Verbindung | Ergebnis    |
| ----------- | ----------- | ---------- | ----------- |
| best effort | best effort | ja         | best effort |
| best effort | reliable    | nein       | -           |
| reliable    | best effort | ja         | best effort |
| reliable    | reliable    | ja         | reliable    |

> **Wichtig**: Ein `reliable` Subscriber kann sich nicht mit einem `best effort` Publisher verbinden!

**QoS-Einstellungen inspizieren:**

```bash
# QoS-Konfiguration eines Topics anzeigen
ros2 topic info /imu/data --verbose

# Beispiel für QoS-Einstellung in Python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

Weitere Informationen: [ROS 2 QoS Documentation](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html)

## Rekapitulation: Vom Bayes-Filter zum Kalman-Filter

--{{0}}--
In der letzten Vorlesung haben wir den diskreten Bayes-Filter kennengelernt. Er funktioniert, hat aber fundamentale Skalierungsprobleme. Heute lernen wir den Kalman-Filter kennen, der diese Probleme elegant löst, indem er eine andere Repräsentation der Unsicherheit verwendet.

In der letzten Vorlesung haben wir den **diskreten Bayes-Filter** implementiert:

+ Zustandsraum wurde in diskrete Zellen unterteilt
+ Für jede Zelle wurde eine Aufenthaltswahrscheinlichkeit gespeichert
+ **Predict**: Faltung mit Bewegungskernel
+ **Update**: Elementweise Multiplikation mit Likelihood

**Problem**: Der Speicher- und Rechenaufwand skaliert mit der Anzahl der Zellen!

<!--
style="width: 80%; min-width: 420px; max-width: 720px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii
    Diskreter Bayes-Filter              Kalman-Filter

    ┌─┬─┬─┬─┬─┬─┬─┬─┬─┬─┐
    │ │▄│█│▄│ │ │ │ │ │ │    ───────►    "$\mu, \sigma^2$"
    └─┴─┴─┴─┴─┴─┴─┴─┴─┴─┘
       n Werte speichern              2 Werte speichern

    100×100 Grid = 10.000 Werte       Immer nur 2 Werte!                                              .
```

--{{0}}--
Die Kernidee ist einfach aber kraftvoll: Anstatt für jede mögliche Position eine Wahrscheinlichkeit zu speichern, nehmen wir an, dass die Unsicherheit normalverteilt ist. Eine Normalverteilung wird vollständig durch zwei Parameter beschrieben: den Mittelwert und die Varianz. Das reduziert den Speicherbedarf von potenziell Millionen von Werten auf konstant zwei.

> **Kernidee**: Wenn wir annehmen, dass alle Unsicherheiten **normalverteilt** sind, reduziert sich die Repräsentation auf nur zwei Parameter: Mittelwert $\mu$ und Varianz $\sigma^2$.

## Warum Normalverteilungen?

       {{0-1}}
*******************************************************************************

--{{0}}--
Warum funktioniert das mit Normalverteilungen so gut? Die Antwort liegt in einer besonderen mathematischen Eigenschaft: Normalverteilungen sind unter Addition und Multiplikation abgeschlossen. Das bedeutet: Wenn wir zwei normalverteilte Zufallsvariablen addieren, ist das Ergebnis wieder normalverteilt. Und wenn wir zwei Normalverteilungen multiplizieren und normalisieren, erhalten wir wieder eine Normalverteilung. Genau diese beiden Operationen brauchen wir für den Predict- und Update-Schritt.

Die Normalverteilung (Gauß-Verteilung) hat besondere mathematische Eigenschaften:

$$
\mathcal{N}(x; \mu, \sigma^2) = \frac{1}{\sqrt{2\pi\sigma^2}} \exp\left(-\frac{(x-\mu)^2}{2\sigma^2}\right)
$$

![Normalverteilung](./images/Normalverteilung.png)<!-- style="width: 70%;"-->

**Wichtige Eigenschaft**: Normalverteilungen sind unter bestimmten Operationen **abgeschlossen**:

1. **Summe** zweier normalverteilter Variablen → wieder normalverteilt
2. **Produkt** zweier Normalverteilungen (nach Normalisierung) → wieder normalverteilt

*******************************************************************************

       {{1-2}}
*******************************************************************************

**Abgeschlossenheit unter Addition (Predict-Schritt)**

--{{1}}--
Schauen wir uns das genauer an. Im Predict-Schritt addieren wir den aktuellen Zustand mit der Bewegung. Wenn beide normalverteilt sind, ist das Ergebnis wieder normalverteilt. Die neuen Parameter berechnen sich einfach: Der neue Mittelwert ist die Summe der Mittelwerte, und die neue Varianz ist die Summe der Varianzen. Das ist der mathematische Grund, warum die Unsicherheit im Predict-Schritt zunimmt.

Wenn $X \sim \mathcal{N}(\mu_X, \sigma_X^2)$ und $Y \sim \mathcal{N}(\mu_Y, \sigma_Y^2)$ unabhängig sind:

$$X + Y \sim \mathcal{N}(\mu_X + \mu_Y, \sigma_X^2 + \sigma_Y^2)$$

→ Die Varianzen **addieren** sich!

```python                          SummeNormalverteilungen.py
import numpy as np
import matplotlib.pyplot as plt

# Zwei Normalverteilungen
mu1, sigma1 = 3, 1
mu2, sigma2 = 2, 0.5

# Summe
mu_sum = mu1 + mu2
sigma_sum = np.sqrt(sigma1**2 + sigma2**2)

x = np.linspace(-2, 12, 500)
y1 = 1/(sigma1*np.sqrt(2*np.pi)) * np.exp(-(x-mu1)**2/(2*sigma1**2))
y2 = 1/(sigma2*np.sqrt(2*np.pi)) * np.exp(-(x-mu2)**2/(2*sigma2**2))
y_sum = 1/(sigma_sum*np.sqrt(2*np.pi)) * np.exp(-(x-mu_sum)**2/(2*sigma_sum**2))

plt.figure(figsize=(10, 5))
plt.plot(x, y1, 'b-', label=f'X ~ N({mu1}, {sigma1}²)', linewidth=2)
plt.plot(x, y2, 'g-', label=f'Y ~ N({mu2}, {sigma2}²)', linewidth=2)
plt.plot(x, y_sum, 'r--', label=f'X+Y ~ N({mu_sum}, {sigma_sum:.2f}²)', linewidth=2)
plt.xlabel('x')
plt.ylabel('Wahrscheinlichkeitsdichte')
plt.title('Summe zweier Normalverteilungen')
plt.legend()
plt.grid(True, alpha=0.3)
plt.savefig('foo.png')
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)

*******************************************************************************

       {{2-3}}
*******************************************************************************

**Abgeschlossenheit unter Multiplikation (Update-Schritt)**

--{{2}}--
Im Update-Schritt kombinieren wir unsere Vorhersage mit einer Messung. Mathematisch entspricht das der Multiplikation zweier Wahrscheinlichkeitsverteilungen. Auch hier bleibt die Normalverteilung erhalten. Die Formeln für den neuen Mittelwert und die neue Varianz sehen komplizierter aus, aber das Prinzip ist klar: Die neue Schätzung liegt zwischen Vorhersage und Messung, gewichtet nach deren jeweiliger Unsicherheit. Wer sicherer ist, hat mehr Einfluss auf das Ergebnis.

Wenn wir zwei Normalverteilungen multiplizieren und normalisieren:

$$
\mathcal{N}(\mu_1, \sigma_1^2) \cdot \mathcal{N}(\mu_2, \sigma_2^2) \propto \mathcal{N}(\mu_{new}, \sigma_{new}^2)
$$

mit:

$$
\mu_{new} = \frac{\mu_1 \sigma_2^2 + \mu_2 \sigma_1^2}{\sigma_1^2 + \sigma_2^2}
\qquad
\sigma_{new}^2 = \frac{\sigma_1^2 \sigma_2^2}{\sigma_1^2 + \sigma_2^2}
$$

→ Die neue Varianz ist **kleiner** als beide Ausgangsvarianzen!

**Grenzfälle zur Intuition:**

| Fall | Ergebnis | Intuition |
|------|----------|-----------|
| $\sigma_1 = \sigma_2 = \sigma$ | $\sigma_{neu}^2 = \frac{\sigma^2}{2}$ | Zwei gleich gute Messungen halbieren die Varianz |
| $\sigma_1 \to 0$ (perfekt) | $\sigma_{neu}^2 \to 0$ | Eine perfekte Messung dominiert |
| $\sigma_1 \to \infty$ (nutzlos) | $\sigma_{neu}^2 \to \sigma_2^2$ | Eine nutzlose Messung ändert nichts |

```python                          ProduktNormalverteilungen.py
import numpy as np
import matplotlib.pyplot as plt

# Prior (Vorhersage) und Likelihood (Messung)
mu_prior, sigma_prior = 5, 2      # Vorhersage: unsicher
mu_meas, sigma_meas = 7, 1        # Messung: genauer

# Posterior nach Bayes-Update
sigma_post_sq = (sigma_prior**2 * sigma_meas**2) / (sigma_prior**2 + sigma_meas**2)
sigma_post = np.sqrt(sigma_post_sq)
mu_post = (mu_prior * sigma_meas**2 + mu_meas * sigma_prior**2) / (sigma_prior**2 + sigma_meas**2)

x = np.linspace(0, 12, 500)
y_prior = 1/(sigma_prior*np.sqrt(2*np.pi)) * np.exp(-(x-mu_prior)**2/(2*sigma_prior**2))
y_meas = 1/(sigma_meas*np.sqrt(2*np.pi)) * np.exp(-(x-mu_meas)**2/(2*sigma_meas**2))
y_post = 1/(sigma_post*np.sqrt(2*np.pi)) * np.exp(-(x-mu_post)**2/(2*sigma_post**2))

plt.figure(figsize=(10, 5))
plt.fill_between(x, y_prior, alpha=0.3, color='blue')
plt.plot(x, y_prior, 'b-', label=f'Prior (Vorhersage): μ={mu_prior}, σ={sigma_prior}', linewidth=2)
plt.fill_between(x, y_meas, alpha=0.3, color='green')
plt.plot(x, y_meas, 'g-', label=f'Likelihood (Messung): μ={mu_meas}, σ={sigma_meas}', linewidth=2)
plt.fill_between(x, y_post, alpha=0.3, color='red')
plt.plot(x, y_post, 'r-', label=f'Posterior: μ={mu_post:.2f}, σ={sigma_post:.2f}', linewidth=2)
plt.xlabel('Position')
plt.ylabel('Wahrscheinlichkeitsdichte')
plt.title('Bayes-Update: Produkt zweier Normalverteilungen')
plt.legend()
plt.grid(True, alpha=0.3)
plt.savefig('foo.png')

print(f"Prior:     μ = {mu_prior:.2f}, σ = {sigma_prior:.2f}")
print(f"Messung:   μ = {mu_meas:.2f}, σ = {sigma_meas:.2f}")
print(f"Posterior: μ = {mu_post:.2f}, σ = {sigma_post:.2f}")
print(f"\nDie Posterior-Varianz ({sigma_post:.2f}²) ist kleiner als beide Eingangsvarianzen!")
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)

--{{2}}--
Beachten Sie das wichtige Ergebnis: Die Posterior-Varianz ist immer kleiner als jede der beiden Eingangsvarianzen! Das macht intuitiv Sinn: Wenn wir zwei unabhängige Informationsquellen kombinieren, werden wir sicherer, nicht unsicherer. Der Posterior liegt näher an der genaueren Messung, weil diese weniger Unsicherheit hat.

*******************************************************************************

## Der eindimensionale Kalman-Filter

--{{0}}--
Jetzt haben wir das mathematische Fundament gelegt und können den Kalman-Filter formal definieren. Er basiert auf zwei Gleichungen: der Systemgleichung, die beschreibt, wie sich der Zustand über die Zeit entwickelt, und der Messgleichung, die beschreibt, wie Sensormessungen mit dem wahren Zustand zusammenhängen. Beide Gleichungen enthalten Rauschterme, die die Unsicherheit modellieren.

Der Kalman-Filter basiert auf zwei grundsätzlichen Gleichungen:

**1. Systemgleichung** (beschreibt die Zustandsentwicklung):

$$x_t = a \cdot x_{t-1} + b \cdot u_t + \epsilon_t$$

**2. Messgleichung** (beschreibt die Sensormessung):

$$z_t = h \cdot x_t + \delta_t$$

Dabei sind:

| Symbol | Bedeutung |
|--------|-----------|
| $x_t$ | Zustand zum Zeitpunkt $t$ |
| $u_t$ | Steuerbefehl (control input) |
| $z_t$ | Messwert |
| $a$ | Systemdynamik (wie entwickelt sich der Zustand?) |
| $b$ | Eingangsmatrix (wie wirkt die Steuerung?) |
| $h$ | Messmatrix (wie hängt Messung vom Zustand ab?) |
| $\epsilon_t$ | Prozessrauschen $\sim \mathcal{N}(0, \sigma_\epsilon^2)$ |
| $\delta_t$ | Messrauschen $\sim \mathcal{N}(0, \sigma_\delta^2)$ |


### Predict-Schritt (Vorhersage)

--{{0}}--
Im Predict-Schritt wenden wir das Systemmodell an, um den nächsten Zustand vorherzusagen. Der neue Mittelwert ergibt sich aus dem alten Zustand transformiert durch die Systemdynamik plus dem Steuerbefehl. Die neue Varianz setzt sich zusammen aus der transformierten alten Varianz plus dem Prozessrauschen. Wichtig: Die Varianz kann nur wachsen, nie schrumpfen. Das spiegelt wider, dass Bewegung immer Unsicherheit hinzufügt.

Mit Hilfe des Systemmodells wird aus dem alten Zustand und einem Steuerbefehl der neue Zustand geschätzt:

![Kalman Prädiktion](./images/KalmanPrediktion.png)<!-- style="width: 60%;"-->

$$
\begin{align*}
\bar{\mu}_t &= a \cdot \mu_{t-1} + b \cdot u_t \\
\bar{\sigma}^2_t &= a^2 \cdot \sigma^2_{t-1} + \sigma^2_\epsilon
\end{align*}
$$

> **Beachte**: Der Querstrich $\bar{\mu}_t$ kennzeichnet die **Vorhersage** (vor dem Update).

Die Unsicherheit wächst durch das Prozessrauschen $\sigma^2_\epsilon$!

### Update-Schritt (Korrektur)

--{{0}}--
Im Update-Schritt korrigieren wir unsere Vorhersage anhand der Messung. Der Schlüssel ist der Kalman-Gain K: Er bestimmt, wie stark wir der Messung vertrauen im Vergleich zur Vorhersage. Wenn die Messung sehr genau ist (kleines Messrauschen), ist K nahe bei 1, und wir vertrauen hauptsächlich der Messung. Wenn die Messung verrauscht ist, ist K klein, und wir bleiben eher bei unserer Vorhersage.

Aus der Differenz zwischen vorhergesagtem und gemessenem Wert (Innovation) wird die Schätzung korrigiert:

![Kalman Korrektur](./images/KalmanKorrektur.png)<!-- style="width: 60%;"-->

$$
\begin{align*}
K_t &= \frac{h \cdot \bar{\sigma}^2_t}{h^2 \cdot \bar{\sigma}^2_t + \sigma^2_\delta} \\[1em]
\mu_t &= \bar{\mu}_t + K_t \cdot (z_t - h \cdot \bar{\mu}_t) \\[0.5em]
\sigma^2_t &= (1 - K_t \cdot h) \cdot \bar{\sigma}^2_t
\end{align*}
$$

| Term | Bedeutung |
|------|-----------|
| $K_t$ | **Kalman-Gain** - Gewichtung zwischen Vorhersage und Messung |
| $z_t - c \cdot \bar{\mu}_t$ | **Innovation** - Differenz zwischen Messung und Erwartung |

--{{1}}--
Die Innovation ist besonders interessant: Sie zeigt uns, wie überraschend die Messung war. Wenn die Innovation null ist, bestätigt die Messung genau unsere Vorhersage, und nichts ändert sich. Je größer die Innovation, desto stärker korrigieren wir.

> **Kalman-Gain interpretiert**:
> - $K \approx 1$: Vertraue hauptsächlich der Messung
> - $K \approx 0$: Vertraue hauptsächlich der Vorhersage

### Der Kalman-Gain im Detail

--{{0}}--
Lassen Sie uns den Kalman-Gain genauer verstehen. Er ist das Verhältnis der Vorhersage-Unsicherheit zur Gesamtunsicherheit. Wenn die Vorhersage sehr unsicher ist, aber die Messung genau, wird K groß - wir vertrauen der Messung mehr. Umgekehrt: Wenn die Vorhersage genau ist, aber die Messung verrauscht, wird K klein. Der Kalman-Filter findet automatisch die optimale Balance.

Der Kalman-Gain ist das Herzstück des Filters:

$$K_t = \frac{\text{Vorhersage-Unsicherheit}}{\text{Vorhersage-Unsicherheit} + \text{Mess-Unsicherheit}}$$

```python                          KalmanGainVisualisierung.py
import numpy as np
import matplotlib.pyplot as plt

# Verschiedene Verhältnisse von Mess- zu Vorhersageunsicherheit
sigma_pred = 2.0  # Vorhersage-Standardabweichung (fest)
sigma_meas_values = np.linspace(0.1, 5, 100)

# Kalman-Gain für h=1
K_values = sigma_pred**2 / (sigma_pred**2 + sigma_meas_values**2)

plt.figure(figsize=(10, 5))
plt.plot(sigma_meas_values, K_values, 'b-', linewidth=2)
plt.axhline(y=0.5, color='gray', linestyle='--', alpha=0.5)
plt.axvline(x=sigma_pred, color='gray', linestyle='--', alpha=0.5)

plt.xlabel('Mess-Standardabweichung σ_δ')
plt.ylabel('Kalman-Gain K')
plt.title(f'Kalman-Gain in Abhängigkeit der Messgenauigkeit (σ_pred = {sigma_pred})')
plt.grid(True, alpha=0.3)

# Annotationen
plt.annotate('Messung genauer → K groß\n(vertraue Messung)',
             xy=(0.5, 0.9), fontsize=10, color='green')
plt.annotate('Messung ungenauer → K klein\n(vertraue Vorhersage)',
             xy=(3.5, 0.2), fontsize=10, color='red')

plt.ylim(0, 1)
plt.savefig('foo.png')
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)


## Praktisches Beispiel: Positionsschätzung

--{{0}}--
Jetzt wird es praktisch! Wir implementieren einen Kalman-Filter für ein einfaches, aber realistisches Szenario: Ein Roboter fährt mit konstanter Geschwindigkeit und misst seine Position mit einem verrauschten GPS-Sensor. Das Bewegungsmodell ist simpel: neue Position gleich alte Position plus Geschwindigkeit mal Zeit. Die Messung gibt uns direkt die Position, aber mit Rauschen überlagert.

**Szenario**: Ein Roboter bewegt sich eindimensional mit konstanter Geschwindigkeit. Er hat:

1. **Odometrie**: Schätzt Bewegung, aber mit Drift (Prozessrauschen)
2. **GPS-Sensor**: Misst Position, aber verrauscht (Messrauschen)

<!--
style="width: 80%; min-width: 420px; max-width: 720px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii

     Bewegungsmodell               Messmodell
    ┌─────────────────┐         ┌─────────────────────┐
    │ x_t = x_{t-1}+v │         │   z_t = x_t         │ 
    │    + ε (Drift)  │         │   + δ (GPS-Rauschen)│
    └─────────────────┘         └─────────────────────┘
           │                                │
           v                                v
    ┌─────────────────────────────────────────────────┐
    │              KALMAN FILTER                      │
    │  Fusioniert Odometrie und GPS optimal!          │
    └─────────────────────────────────────────────────┘                                                           .
```

**Parameter für unser Beispiel**:

- $a = 1$ (Position bleibt erhalten)
- $b = \Delta t = 1$ (Zeitschritt)
- $h = 1$ (GPS misst Position direkt)
- $\sigma_\epsilon = 0.5$ (Odometrie-Drift)
- $\sigma_\delta = 2.0$ (GPS-Rauschen)


### Implementierung des 1D Kalman-Filters

--{{0}}--
Hier ist die vollständige Implementierung. Der Code ist überraschend kurz! Der Predict-Schritt besteht aus zwei Zeilen, der Update-Schritt aus drei. Wir simulieren einen Roboter, der sich mit Geschwindigkeit 1 bewegt. Die wahre Position kennen wir nur in der Simulation - in der Realität hätten wir diese nicht. Wir haben nur die verrauschten GPS-Messungen und die Odometrie. Der Filter kombiniert beide optimal.

```python                          KalmanFilter1D.py
import numpy as np
import matplotlib.pyplot as plt

# Simulationsparameter
np.random.seed(42)
n_steps = 50
dt = 1.0

# Systemparameter
a = 1.0          # Zustandstransition
b = dt           # Steuerungseinfluss
h = 1.0          # Messmatrix
velocity = 1.0   # Konstante Geschwindigkeit (Steuerbefehl u)

# Rauschparameter
sigma_process = 0.5   # Prozessrauschen (Odometrie-Drift)
sigma_meas = 2.0      # Messrauschen (GPS)

# Wahre Position (nur für Simulation bekannt)
true_position = np.zeros(n_steps)
for t in range(1, n_steps):
    true_position[t] = true_position[t-1] + velocity * dt

# Verrauschte Messungen (GPS)
measurements = true_position + np.random.normal(0, sigma_meas, n_steps)

# Kalman-Filter
mu = np.zeros(n_steps)       # Geschätzte Position
sigma_sq = np.zeros(n_steps) # Geschätzte Varianz

# Initialisierung
mu[0] = measurements[0]      # Starte bei erster Messung
sigma_sq[0] = sigma_meas**2  # Initiale Unsicherheit

for t in range(1, n_steps):
    # === PREDICT ===
    mu_pred = a * mu[t-1] + b * velocity
    sigma_sq_pred = a**2 * sigma_sq[t-1] + sigma_process**2

    # === UPDATE ===
    K = (h * sigma_sq_pred) / (h**2 * sigma_sq_pred + sigma_meas**2)
    mu[t] = mu_pred + K * (measurements[t] - h * mu_pred)
    sigma_sq[t] = (1 - K * h) * sigma_sq_pred

# Visualisierung
fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

# Oberes Plot: Positionen
ax1 = axes[0]
ax1.plot(true_position, 'k-', linewidth=2, label='Wahre Position')
ax1.scatter(range(n_steps), measurements, c='red', s=20, alpha=0.5, label='GPS-Messungen')
ax1.plot(mu, 'b-', linewidth=2, label='Kalman-Schätzung')
ax1.fill_between(range(n_steps),
                  mu - 2*np.sqrt(sigma_sq),
                  mu + 2*np.sqrt(sigma_sq),
                  color='blue', alpha=0.2, label='95% Konfidenzintervall')
ax1.set_ylabel('Position')
ax1.legend(loc='upper left')
ax1.set_title('1D Kalman-Filter: Positionsschätzung')
ax1.grid(True, alpha=0.3)

# Unteres Plot: Unsicherheit und Kalman-Gain
ax2 = axes[1]
ax2.plot(np.sqrt(sigma_sq), 'b-', linewidth=2, label='Standardabweichung σ')
ax2.axhline(y=sigma_meas, color='red', linestyle='--', label=f'GPS-Rauschen (σ={sigma_meas})')
ax2.axhline(y=sigma_process, color='green', linestyle='--', label=f'Prozessrauschen (σ={sigma_process})')
ax2.set_xlabel('Zeitschritt')
ax2.set_ylabel('Standardabweichung')
ax2.legend(loc='upper right')
ax2.set_title('Entwicklung der Schätzunsicherheit')
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('foo.png')

# Fehleranalyse
estimation_error = np.abs(mu - true_position)
measurement_error = np.abs(measurements - true_position)
print(f"Mittlerer GPS-Fehler:      {np.mean(measurement_error):.3f}")
print(f"Mittlerer Kalman-Fehler:   {np.mean(estimation_error):.3f}")
print(f"Verbesserung:              {(1 - np.mean(estimation_error)/np.mean(measurement_error))*100:.1f}%")
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)

--{{1}}--
Schauen Sie sich die Ergebnisse an! Die roten Punkte sind die verrauschten GPS-Messungen, die stark um die wahre Position streuen. Die blaue Linie ist die Kalman-Schätzung - sie folgt der wahren Position viel glatter und genauer. Das hellblaue Band zeigt das 95%-Konfidenzintervall. Im unteren Plot sehen Sie, wie die Unsicherheit nach wenigen Schritten konvergiert. Der Kalman-Filter findet automatisch das optimale Gleichgewicht zwischen Prozess- und Messrauschen.


### Variation der Parameter

--{{0}}--
Um den Filter besser zu verstehen, experimentieren wir mit verschiedenen Parametern. Was passiert, wenn wir das Messrauschen verändern? Ein genauerer GPS-Sensor führt zu einem höheren Kalman-Gain und einer schnelleren Konvergenz. Ein ungenauerer Sensor bedeutet, dass der Filter mehr auf das Bewegungsmodell vertraut.

Experimentieren Sie mit verschiedenen Rauschparametern:

```python                          KalmanParameterVariation.py
import numpy as np
import matplotlib.pyplot as plt

np.random.seed(42)
n_steps = 30

# Wahre Position
true_pos = np.cumsum(np.ones(n_steps))

# Verschiedene Szenarien
scenarios = [
    {"sigma_proc": 0.1, "sigma_meas": 2.0, "name": "Gute Odometrie, schlechtes GPS"},
    {"sigma_proc": 1.0, "sigma_meas": 0.5, "name": "Schlechte Odometrie, gutes GPS"},
    {"sigma_proc": 0.5, "sigma_meas": 0.5, "name": "Beide gleich gut"},
]

fig, axes = plt.subplots(1, 3, figsize=(15, 4))

for idx, scenario in enumerate(scenarios):
    sigma_proc = scenario["sigma_proc"]
    sigma_meas = scenario["sigma_meas"]

    # Messungen generieren
    measurements = true_pos + np.random.normal(0, sigma_meas, n_steps)

    # Kalman-Filter
    mu = np.zeros(n_steps)
    mu[0] = measurements[0]
    sigma_sq = sigma_meas**2

    kalman_gains = []

    for t in range(1, n_steps):
        # Predict
        mu_pred = mu[t-1] + 1.0
        sigma_sq_pred = sigma_sq + sigma_proc**2

        # Update
        K = sigma_sq_pred / (sigma_sq_pred + sigma_meas**2)
        kalman_gains.append(K)
        mu[t] = mu_pred + K * (measurements[t] - mu_pred)
        sigma_sq = (1 - K) * sigma_sq_pred

    # Plot
    ax = axes[idx]
    ax.plot(true_pos, 'k-', linewidth=2, label='Wahr')
    ax.scatter(range(n_steps), measurements, c='red', s=15, alpha=0.5, label='Messung')
    ax.plot(mu, 'b-', linewidth=2, label='Kalman')
    ax.set_title(f"{scenario['name']}\nK ≈ {np.mean(kalman_gains):.2f}")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('foo.png')
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)

--{{1}}--
Im linken Plot haben wir gute Odometrie aber schlechtes GPS. Der Kalman-Gain ist klein, der Filter vertraut hauptsächlich dem Bewegungsmodell und glättet die verrauschten Messungen stark. Im mittleren Plot ist es umgekehrt: schlechte Odometrie, gutes GPS. Der Kalman-Gain ist groß, und der Filter folgt den Messungen eng. Im rechten Plot sind beide gleich gut, und der Filter balanciert zwischen beiden Quellen.

## Der mehrdimensionale Kalman-Filter

--{{0}}--
In der Praxis haben wir selten nur eine Zustandsvariable. Ein Roboter hat Position und Geschwindigkeit, vielleicht auch Beschleunigung und Orientierung. Der mehrdimensionale Kalman-Filter erweitert die Idee auf Vektoren und Matrizen. Die Prinzipien bleiben gleich, aber statt Skalaren arbeiten wir jetzt mit Matrizen.

In realen Anwendungen ist der Zustand mehrdimensional. Beispiel: Position **und** Geschwindigkeit schätzen.

$$
\mathbf{x}_t = \begin{pmatrix} x \\ \dot{x} \end{pmatrix} = \begin{pmatrix} \text{Position} \\ \text{Geschwindigkeit} \end{pmatrix}
$$

Die Systemgleichungen werden zu Matrixgleichungen:

$$
\begin{align*}
\mathbf{x}_t &= \mathbf{A} \cdot \mathbf{x}_{t-1} + \mathbf{B} \cdot \mathbf{u}_t + \boldsymbol{\epsilon}_t \\
\mathbf{z}_t &= \mathbf{H} \cdot \mathbf{x}_t + \boldsymbol{\delta}_t
\end{align*}
$$

| Symbol | Dimension | Bedeutung |
|--------|-----------|-----------|
| $\mathbf{x}$ | $n \times 1$ | Zustandsvektor |
| $\mathbf{A}$ | $n \times n$ | Systemmatrix |
| $\mathbf{B}$ | $n \times m$ | Steuermatrix |
| $\mathbf{H}$ | $k \times n$ | Messmatrix |
| $\mathbf{Q}$ | $n \times n$ | Prozessrausch-Kovarianz |
| $\mathbf{R}$ | $k \times k$ | Messrausch-Kovarianz |
| $\mathbf{P}$ | $n \times n$ | Schätz-Kovarianzmatrix |


### Kalman-Filter Gleichungen in Matrixform

--{{0}}--
Hier sind die vollständigen Kalman-Filter Gleichungen. Im Predict-Schritt transformieren wir den Zustand mit der Systemmatrix und addieren die Steuerung. Die Kovarianzmatrix wird transformiert und das Prozessrauschen addiert. Im Update-Schritt berechnen wir erst den Kalman-Gain als Matrix, dann die Innovation, und schließlich aktualisieren wir Zustand und Kovarianz.

**Predict-Schritt:**

$$
\begin{align*}
\bar{\mathbf{x}}_t &= \mathbf{A} \cdot \mathbf{x}_{t-1} + \mathbf{B} \cdot \mathbf{u}_t \\
\bar{\mathbf{P}}_t &= \mathbf{A} \cdot \mathbf{P}_{t-1} \cdot \mathbf{A}^T + \mathbf{Q}
\end{align*}
$$

**Update-Schritt:**

$$
\begin{align*}
\mathbf{K}_t &= \bar{\mathbf{P}}_t \cdot \mathbf{H}^T \cdot (\mathbf{H} \cdot \bar{\mathbf{P}}_t \cdot \mathbf{H}^T + \mathbf{R})^{-1} \\
\mathbf{x}_t &= \bar{\mathbf{x}}_t + \mathbf{K}_t \cdot (\mathbf{z}_t - \mathbf{H} \cdot \bar{\mathbf{x}}_t) \\
\mathbf{P}_t &= (\mathbf{I} - \mathbf{K}_t \cdot \mathbf{H}) \cdot \bar{\mathbf{P}}_t
\end{align*}
$$

![Kalman-Filter Ablauf](./images/Kalman.png "_S. Maybeck, Stochastic models, estimation, and control, 1977_")<!-- style="width: 80%;"-->


### Beispiel: Position und Geschwindigkeit schätzen

--{{0}}--
Ein praktisches Beispiel: Wir wollen Position und Geschwindigkeit eines Objekts gleichzeitig schätzen, obwohl unser Sensor nur die Position misst. Das klingt zunächst unmöglich - wie können wir die Geschwindigkeit schätzen, ohne sie zu messen? Der Trick liegt in der Systemdynamik: Wenn wir wissen, wie sich Position und Geschwindigkeit zueinander verhalten, können wir aus den Positionsänderungen auf die Geschwindigkeit schließen.

```python                          KalmanFilter2D.py
import numpy as np
import matplotlib.pyplot as plt

np.random.seed(42)
n_steps = 100
dt = 0.1

# Systemmatrizen für konstante Geschwindigkeit
A = np.array([[1, dt],    # x_new = x + v*dt
              [0, 1]])    # v_new = v

B = np.array([[0.5*dt**2],  # Beschleunigungseinfluss auf Position
              [dt]])         # Beschleunigungseinfluss auf Geschwindigkeit

H = np.array([[1, 0]])    # Wir messen nur die Position!

# Rauschmatrizen
Q = np.array([[0.1, 0],   # Prozessrauschen
              [0, 0.1]])
R = np.array([[1.0]])     # Messrauschen

# Wahre Trajektorie: Start, Beschleunigung, konstant, Bremsen
true_state = np.zeros((n_steps, 2))
true_state[0] = [0, 0]

for t in range(1, n_steps):
    if t < 30:
        u = 2.0    # Beschleunigen
    elif t < 70:
        u = 0.0    # Konstant
    else:
        u = -1.5   # Bremsen

    true_state[t] = A @ true_state[t-1] + B.flatten() * u

# Messungen (nur Position, verrauscht)
measurements = true_state[:, 0] + np.random.normal(0, np.sqrt(R[0,0]), n_steps)

# Kalman-Filter
x_est = np.zeros((n_steps, 2))
P = np.eye(2) * 10  # Hohe initiale Unsicherheit

x_est[0] = [measurements[0], 0]  # Starte mit erster Messung, Geschwindigkeit unbekannt

for t in range(1, n_steps):
    # Steuerung (in Realität oft unbekannt, hier setzen wir u=0)
    u = 0

    # Predict
    x_pred = A @ x_est[t-1] + B.flatten() * u
    P_pred = A @ P @ A.T + Q

    # Update
    S = H @ P_pred @ H.T + R  # Innovation-Kovarianz
    K = P_pred @ H.T @ np.linalg.inv(S)

    innovation = measurements[t] - H @ x_pred
    x_est[t] = x_pred + K.flatten() * innovation
    P = (np.eye(2) - K @ H) @ P_pred

# Visualisierung
fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

# Position
ax1 = axes[0]
ax1.plot(true_state[:, 0], 'k-', linewidth=2, label='Wahre Position')
ax1.scatter(range(n_steps), measurements, c='red', s=10, alpha=0.3, label='Messungen')
ax1.plot(x_est[:, 0], 'b-', linewidth=2, label='Kalman-Schätzung')
ax1.set_ylabel('Position')
ax1.legend()
ax1.set_title('2D Kalman-Filter: Position und Geschwindigkeit')
ax1.grid(True, alpha=0.3)

# Geschwindigkeit (wird NICHT gemessen, aber geschätzt!)
ax2 = axes[1]
ax2.plot(true_state[:, 1], 'k-', linewidth=2, label='Wahre Geschwindigkeit')
ax2.plot(x_est[:, 1], 'b-', linewidth=2, label='Geschätzte Geschwindigkeit')
ax2.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
ax2.set_xlabel('Zeitschritt')
ax2.set_ylabel('Geschwindigkeit')
ax2.legend()
ax2.set_title('Geschwindigkeit wird aus Positionsmessungen inferiert!')
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('foo.png')

print("Beachte: Die Geschwindigkeit wird NICHT gemessen,")
print("aber der Kalman-Filter kann sie aus den Positionsänderungen schätzen!")
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)

--{{1}}--
Das Ergebnis ist beeindruckend: Obwohl wir nur die Position messen, kann der Kalman-Filter die Geschwindigkeit ziemlich gut schätzen! Er erkennt die Beschleunigungsphase, die Phase konstanter Geschwindigkeit und das Abbremsen. Das liegt daran, dass Positionsänderungen Information über die Geschwindigkeit enthalten. Der Filter nutzt diese implizite Information optimal aus.

## Extended Kalman Filter (EKF)

--{{0}}--
Der klassische Kalman-Filter hat eine wichtige Einschränkung: Er funktioniert nur für lineare Systeme. Aber in der Robotik sind die meisten Systeme nichtlinear! Wenn ein Roboter sich dreht, ist die Beziehung zwischen Steuerbefehl und neuer Position nichtlinear. Sensoren wie Lidar oder Kameras haben ebenfalls nichtlineare Messmodelle. Der Extended Kalman Filter löst dieses Problem durch Linearisierung.

Der klassische Kalman-Filter setzt **lineare** System- und Messmodelle voraus. In der Robotik sind diese jedoch oft **nichtlinear**:

**Beispiele für Nichtlinearitäten:**

- **Kinematik eines Differential-Drive-Roboters:** Das Bewegungsmodell enthält trigonometrische Funktionen des Orientierungswinkels $\theta$. Die $\sin$- und $\cos$-Terme ($v$ ist die Translationsgeschwindigkeit (linear, vorwärts) des Roboters $\omega$ ist die Rotationsgeschwindigkeit (Drehrate) des Roboters) machen das Systemmodell $g(\mathbf{x}, \mathbf{u})$ nichtlinear:

$$
\begin{pmatrix} x' \\ y' \\ \theta' \end{pmatrix} = \begin{pmatrix} x \\ y \\ \theta \end{pmatrix} + \begin{pmatrix} \cos(\theta) & 0 \\ \sin(\theta) & 0 \\ 0 & 1 \end{pmatrix} \begin{pmatrix} v \\ \omega \end{pmatrix} \Delta t
$$

- **Transformation zwischen Polarkoordinaten (Lidar) und kartesischen Koordinaten:** Lidar liefert Messwerte in Polarkoordinaten $(r, \varphi)$, der Zustandsraum ist aber kartesisch. Die Umrechnung $x = r \cdot \cos(\varphi)$, $y = r \cdot \sin(\varphi)$ bzw. umgekehrt $r = \sqrt{x^2 + y^2}$, $\varphi = \text{atan2}(y, x)$ ist nichtlinear (Wurzel, Arkustangens).

$$
\begin{align*}
\mathbf{x}_t &= g(\mathbf{x}_{t-1}, \mathbf{u}_t) + \boldsymbol{\epsilon}_t \\
\mathbf{z}_t &= h(\mathbf{x}_t) + \boldsymbol{\delta}_t
\end{align*}
$$

In allen Fällen sind $g(\cdot)$ und $h(\cdot)$ **nichtlineare Funktionen**!

### Linearisierung durch Taylor-Entwicklung

--{{0}}--
Die Idee des EKF ist elegant: Wir linearisieren die nichtlinearen Funktionen lokal um den aktuellen Schätzwert. Das machen wir mit einer Taylor-Entwicklung erster Ordnung, also mit den Jacobi-Matrizen. Die Jacobi-Matrix enthält alle partiellen Ableitungen und beschreibt, wie sich kleine Änderungen im Eingang auf den Ausgang auswirken.

Der EKF linearisiert die nichtlinearen Funktionen um den aktuellen Schätzwert:

$$
g(\mathbf{x}) \approx g(\boldsymbol{\mu}) + \underbrace{\frac{\partial g}{\partial \mathbf{x}}\bigg|_{\boldsymbol{\mu}}}_{\mathbf{G}} (\mathbf{x} - \boldsymbol{\mu})
$$

Die **Jacobi-Matrizen** ersetzen die Systemmatrizen:

$$
\mathbf{G}_t = \frac{\partial g}{\partial \mathbf{x}}\bigg|_{\mathbf{x}_{t-1}, \mathbf{u}_t}
\qquad
\mathbf{H}_t = \frac{\partial h}{\partial \mathbf{x}}\bigg|_{\bar{\mathbf{x}}_t}
$$


### EKF-Algorithmus

--{{0}}--
Der EKF-Algorithmus ist fast identisch zum Standard-Kalman-Filter. Der Unterschied: Im Predict-Schritt wenden wir die nichtlineare Funktion g direkt an für den Zustand, aber verwenden die Jacobi-Matrix G für die Kovarianz. Im Update-Schritt verwenden wir die nichtlineare Messfunktion h für die erwartete Messung, aber die Jacobi-Matrix H für die Kovarianz-Updates.

**Predict-Schritt:**

$$
\begin{align*}
\bar{\mathbf{x}}_t &= g(\mathbf{x}_{t-1}, \mathbf{u}_t) \\
\bar{\mathbf{P}}_t &= \mathbf{G}_t \cdot \mathbf{P}_{t-1} \cdot \mathbf{G}_t^T + \mathbf{Q}
\end{align*}
$$

**Update-Schritt:**

$$
\begin{align*}
\mathbf{K}_t &= \bar{\mathbf{P}}_t \cdot \mathbf{H}_t^T \cdot (\mathbf{H}_t \cdot \bar{\mathbf{P}}_t \cdot \mathbf{H}_t^T + \mathbf{R})^{-1} \\
\mathbf{x}_t &= \bar{\mathbf{x}}_t + \mathbf{K}_t \cdot (\mathbf{z}_t - h(\bar{\mathbf{x}}_t)) \\
\mathbf{P}_t &= (\mathbf{I} - \mathbf{K}_t \cdot \mathbf{H}_t) \cdot \bar{\mathbf{P}}_t
\end{align*}
$$


> **Beachte**: Die nichtlinearen Funktionen $g(\cdot)$ und $h(\cdot)$ werden nur für die **Zustandsfortschreibung** direkt ausgewertet. Für die **Kovarianz-Propagation** werden stattdessen die Jacobi-Matrizen $\mathbf{G}_t$ und $\mathbf{H}_t$ benötigt, da die Kovarianz eine Matrixmultiplikation erfordert ($\mathbf{G} \mathbf{P} \mathbf{G}^T$) und nicht direkt durch eine nichtlineare Funktion transformiert werden kann. Die Jacobi-Matrizen beschreiben, wie sich kleine Abweichungen vom geschätzten Zustand durch die Nichtlinearität hindurch verzerren.

> **Wichtig**: Der EKF ist eine Approximation! Bei stark nichtlinearen Systemen kann er versagen.


### Beispiel: Roboter mit Winkel

--{{0}}--
Ein typisches Robotik-Beispiel: Ein Roboter hat Position x, y und Orientierung theta. Die Kinematik ist nichtlinear, weil Sinus und Kosinus vorkommen. Wenn der Roboter vorwärts fährt, hängt die Änderung von x und y vom aktuellen Winkel ab. Für den EKF berechnen wir die Jacobi-Matrix, die beschreibt, wie kleine Änderungen in der Geschwindigkeit und Drehrate den Zustand beeinflussen.

                {{0-2}}
******************************************************

**Zustandsvektor und Steuereingang:**

$$
\mathbf{x} = \begin{pmatrix} x \\ y \\ \theta \end{pmatrix}, \qquad \mathbf{u} = \begin{pmatrix} v \\ \omega \end{pmatrix}
$$

**Nichtlineares Bewegungsmodell** $g(\mathbf{x}, \mathbf{u})$:

$$
g(\mathbf{x}, \mathbf{u}) = \begin{pmatrix} x + \frac{v}{\omega} \left( \sin(\theta + \omega \Delta t) - \sin(\theta) \right) \\ y + \frac{v}{\omega} \left( -\cos(\theta + \omega \Delta t) + \cos(\theta) \right) \\ \theta + \omega \Delta t \end{pmatrix}
$$

**Jacobi-Matrix** $\mathbf{G}_t = \frac{\partial g}{\partial \mathbf{x}}$:

$$
\mathbf{G}_t = \begin{pmatrix} 1 & 0 & \frac{v}{\omega} \left( \cos(\theta + \omega \Delta t) - \cos(\theta) \right) \\ 0 & 1 & \frac{v}{\omega} \left( \sin(\theta + \omega \Delta t) - \sin(\theta) \right) \\ 0 & 0 & 1 \end{pmatrix}
$$

> **Hinweis:** Für Geradeausfahrt ($\omega \approx 0$) ist der Term $\frac{v}{\omega}$ numerisch instabil. In der Implementierung wird dieser Fall separat über den Grenzwert $\omega \to 0$ behandelt, bei dem das Modell zu $x' = x + v \Delta t \cos(\theta)$, $y' = y + v \Delta t \sin(\theta)$ vereinfacht.

******************************************************

                {{1-2}}
******************************************************

```python                          EKF_Robot.py
import numpy as np
import matplotlib.pyplot as plt

np.random.seed(42)

def motion_model(x, u, dt):
    """Nichtlineares Bewegungsmodell für Differential-Drive"""
    theta = x[2]
    v, omega = u

    if abs(omega) < 1e-6:  # Geradeausfahrt
        x_new = x[0] + v * dt * np.cos(theta)
        y_new = x[1] + v * dt * np.sin(theta)
        theta_new = theta
    else:  # Kurvenfahrt
        x_new = x[0] + v/omega * (np.sin(theta + omega*dt) - np.sin(theta))
        y_new = x[1] + v/omega * (-np.cos(theta + omega*dt) + np.cos(theta))
        theta_new = theta + omega * dt

    return np.array([x_new, y_new, theta_new])

def jacobian_motion(x, u, dt):
    """Jacobi-Matrix des Bewegungsmodells"""
    theta = x[2]
    v, omega = u

    G = np.eye(3)
    if abs(omega) < 1e-6:
        G[0, 2] = -v * dt * np.sin(theta)
        G[1, 2] = v * dt * np.cos(theta)
    else:
        G[0, 2] = v/omega * (np.cos(theta + omega*dt) - np.cos(theta))
        G[1, 2] = v/omega * (np.sin(theta + omega*dt) - np.sin(theta))

    return G

# Simulation
n_steps = 100
dt = 0.1

# Wahre Trajektorie (Kreis fahren)
true_state = np.zeros((n_steps, 3))
v_true = 1.0      # Geschwindigkeit
omega_true = 0.3  # Drehrate

for t in range(1, n_steps):
    u = np.array([v_true, omega_true])
    true_state[t] = motion_model(true_state[t-1], u, dt)

# GPS-Messungen (nur x, y)
sigma_gps = 0.5
measurements = true_state[:, :2] + np.random.normal(0, sigma_gps, (n_steps, 2))

# EKF
x_est = np.zeros((n_steps, 3))
x_est[0] = [measurements[0, 0], measurements[0, 1], 0]

P = np.diag([1, 1, 0.1])
Q = np.diag([0.0001, 0.0001, 0.001])  # Prozessrauschen (Modellvertrauen)
R = np.diag([sigma_gps**2, sigma_gps**2])  # Messrauschen (Sensorvertrauen)
H = np.array([[1, 0, 0],
              [0, 1, 0]])

for t in range(1, n_steps):
    u = np.array([v_true, omega_true])  # Bekannte Steuerung

    # EKF Predict
    x_pred = motion_model(x_est[t-1], u, dt)
    G = jacobian_motion(x_est[t-1], u, dt)
    P_pred = G @ P @ G.T + Q

    # EKF Update
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)

    innovation = measurements[t] - H @ x_pred
    x_est[t] = x_pred + K @ innovation
    P = (np.eye(3) - K @ H) @ P_pred

# Visualisierung
plt.figure(figsize=(10, 10))
plt.plot(true_state[:, 0], true_state[:, 1], 'k-', linewidth=2, label='Wahre Trajektorie')
plt.scatter(measurements[:, 0], measurements[:, 1], c='red', s=10, alpha=0.3, label='GPS-Messungen')
plt.plot(x_est[:, 0], x_est[:, 1], 'b-', linewidth=2, label='EKF-Schätzung')

# Orientierungspfeile
for i in range(0, n_steps, 10):
    dx = 0.3 * np.cos(x_est[i, 2])
    dy = 0.3 * np.sin(x_est[i, 2])
    plt.arrow(x_est[i, 0], x_est[i, 1], dx, dy, head_width=0.1, color='blue', alpha=0.5)

plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('Extended Kalman Filter: Roboter-Lokalisierung')
plt.legend()
plt.axis('equal')
plt.grid(True, alpha=0.3)
plt.savefig('foo.png')

print("Der EKF schätzt auch die Orientierung θ,")
print("obwohl GPS nur x und y misst!")
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)

********************************************************

--{{2}}--
Das Ergebnis zeigt die Leistungsfähigkeit des EKF: Obwohl GPS nur x und y liefert, kann der Filter auch die Orientierung theta schätzen! Die blauen Pfeile zeigen die geschätzte Blickrichtung. Der EKF nutzt das Wissen über die Roboterkinematik, um aus den Positionsänderungen auf die Orientierung zu schließen.

                    {{2-3}}
*****************************************************

> **Übung: Parameter-Tuning** — Verändern Sie die Kovarianzmatrizen $\mathbf{Q}$ und $\mathbf{R}$ im Code und beobachten Sie die Auswirkungen:
>
> | Parameter | Bedeutung | Auswirkung wenn zu groß | Auswirkung wenn zu klein |
> |---|---|---|---|
> | $\mathbf{Q}$ (Prozessrauschen) | Vertrauen ins Bewegungsmodell | Filter misstraut dem Modell → folgt verrauschten Messungen → Schätzung schwankt | Filter vertraut dem Modell blind → ignoriert Messungen → Schätzung driftet bei Modellfehlern |
> | $\mathbf{R}$ (Messrauschen) | Vertrauen in die Sensoren | Filter ignoriert Messungen → verhält sich wie reine Vorhersage | Filter folgt Messungen zu stark → Rauschen schlägt durch |
>
> **Faustregel:** $\mathbf{Q}$ und $\mathbf{R}$ bestimmen das Verhältnis von Modell- zu Messvertrauen. Nicht die absoluten Werte zählen, sondern das **Verhältnis** $\mathbf{Q}$ zu $\mathbf{R}$. Versuchen Sie z.B. `Q = np.diag([0.01, 0.01, 0.01])` und vergleichen Sie das Ergebnis mit den aktuellen Werten.

*****************************************************

## robot_localization in ROS 2

--{{0}}--
Das Paket robot_localization ist die Standard-Lösung für Sensorfusion in ROS. Es implementiert sowohl einen EKF als auch ein Unscented Kalman Filter und kann beliebig viele Sensoren fusionieren. Sie müssen den Filter nicht selbst implementieren - Sie konfigurieren nur, welche Sensoren welche Zustandsvariablen beeinflussen.


Das ROS 2 Paket `robot_localization` bietet Implementierungen von:

- **Extended Kalman Filter (EKF)**: `ekf_node`
- **Unscented Kalman Filter (UKF)**: `ukf_node`

> **UKF vs. EKF:** Der Unscented Kalman Filter verfolgt einen anderen Ansatz als der EKF. Statt die nichtlineare Funktion durch eine Jacobi-Matrix zu linearisieren, wählt der UKF gezielt sogenannte **Sigma-Punkte** um den aktuellen Schätzwert aus und propagiert diese direkt durch die nichtlineare Funktion $g(\cdot)$ bzw. $h(\cdot)$. Aus den transformierten Punkten werden Mittelwert und Kovarianz rekonstruiert. Der Vorteil: Es müssen **keine Jacobi-Matrizen berechnet** werden, und die Approximation ist bei stark nichtlinearen Systemen oft genauer (bis zur 2. Ordnung statt nur 1. Ordnung beim EKF).

https://docs.ros.org/en/melodic/api/robot_localization/html/index.html

!?[](https://vimeo.com/142624091)


**Unterstützte Sensoren:**

<!-- data-type="none" -->
| Sensor-Typ | ROS Message                               | Typische Quelle              |
| ---------- | ----------------------------------------- | ---------------------------- |
| Odometrie  | `nav_msgs/Odometry`                       | Rad-Encoder, Visual Odometry |
| IMU        | `sensor_msgs/Imu`                         | Beschleunigung, Drehrate     |
| GPS        | `sensor_msgs/NavSatFix`                   | GNSS-Empfänger               |
| Pose       | `geometry_msgs/PoseWithCovarianceStamped` | AMCL, Marker-Tracking        |

```bash
# Installation
sudo apt install ros-${ROS_DISTRO}-robot-localization
```

### Konfiguration des EKF-Nodes

--{{0}}--
Die Konfiguration erfolgt über eine YAML-Datei. Der wichtigste Teil ist die Angabe, welche Komponenten jeder Sensor beiträgt. Der 15-dimensionale Zustandsvektor enthält Position, Orientierung, lineare und Winkelgeschwindigkeit sowie lineare Beschleunigung. Für jeden Sensor geben Sie eine 15-elementige Liste an, die angibt, ob dieser Sensor die jeweilige Komponente beeinflusst.

```yaml
# config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    # Frequenz des Filters
    frequency: 30.0

    # Koordinatensystem
    world_frame: odom
    odom_frame: odom
    base_link_frame: base_link

    # Erster Sensor: Rad-Odometrie
    odom0: /wheel_odom
    odom0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   true,  true,  false,   # vx, vy, vz
                   false, false, true,    # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az

    # Zweiter Sensor: IMU
    imu0: /imu/data
    imu0_config: [false, false, false,   # Position nicht von IMU
                  true,  true,  true,    # Orientierung
                  false, false, false,   # Geschwindigkeit nicht direkt
                  true,  true,  true,    # Winkelgeschwindigkeit
                  true,  true,  true]    # Beschleunigung

    # Prozessrauschen (Tuning-Parameter)
    process_noise_covariance: [0.05, 0, 0, ...]  # 15x15 Matrix
```

--{{1}}--
In diesem Beispiel fusionieren wir Rad-Odometrie und IMU. Die Odometrie liefert x, y Position, yaw-Orientierung und die entsprechenden Geschwindigkeiten. Die IMU liefert die volle Orientierung, Winkelgeschwindigkeiten und Beschleunigungen. Der EKF kombiniert beide optimal.

### Launch-Datei und Integration

--{{0}}--
Die Integration in Ihr ROS 2 System erfolgt über eine Launch-Datei. Sie starten den EKF-Node mit Ihrer Konfiguration und remappen die Topics entsprechend Ihrer Sensor-Setup. Der Filter publiziert die gefilterte Odometrie und optional auch Transformationen für das tf2-System.

```python
# launch/ekf.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_robot'),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config],
            remappings=[
                ('/odometry/filtered', '/odom'),
            ]
        )
    ])
```

**Typische Systemarchitektur:**

<!--
style="width: 90%; min-width: 420px; max-width: 820px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│ Rad-Encoder │     │     IMU     │     │    GPS      │
└──────┬──────┘     └──────┬──────┘     └──────┬──────┘
       │                   │                   │
       ▼                   ▼                   ▼
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│ /wheel_odom │     │  /imu/data  │     │  /gps/fix   │
└──────┬──────┘     └──────┬──────┘     └──────┬──────┘
       │                   │                   │
       └───────────────────┼───────────────────┘
                           │
                           ▼
                ┌─────────────────────┐
                │  robot_localization │
                │     (EKF Node)      │
                └──────────┬──────────┘
                           │
                           ▼
                ┌─────────────────────┐
                │  /odometry/filtered │
                │     (gefiltert)     │
                └─────────────────────┘                                                                 .
```

### Tipps für die Praxis

--{{0}}--
Zum Abschluss einige praktische Tipps. Die Kovarianz-Werte in den Sensor-Messages sind entscheidend - sie sagen dem Filter, wie sehr er jedem Sensor vertrauen soll. Beginnen Sie mit konservativen Werten und tunen Sie dann. Das Prozessrauschen bestimmt, wie stark der Filter auf neue Messungen reagiert. Zu wenig: träge Reaktion. Zu viel: verrauschte Ausgabe.

> **Kovarianz-Werte sind entscheidend!**

1. **Sensor-Kovarianzen**: Stellen Sie sicher, dass Ihre Sensoren realistische Kovarianz-Werte in ihren Messages publizieren

2. **Prozessrauschen tunen**:
   - Zu klein → Filter reagiert zu langsam
   - Zu groß → Schätzung wird verrauscht

3. **Debugging-Tools**:
   ```bash
   # Visualisierung in RViz
   ros2 run rviz2 rviz2
   # Topics inspizieren
   ros2 topic echo /odometry/filtered
   ```

4. **Typische Probleme**:
   - Zeitstempel nicht synchronisiert → `use_sim_time` prüfen
   - Falsche Frame-IDs → tf-Tree überprüfen
   - Kovarianz = 0 → Sensor wird ignoriert!

## Zusammenfassung

--{{0}}--
Fassen wir zusammen, was wir heute gelernt haben. Der Kalman-Filter ist ein optimaler Schätzer für lineare Systeme mit Gauß'schem Rauschen. Er nutzt die besondere Eigenschaft von Normalverteilungen, um mit nur zwei Parametern pro Dimension auszukommen. Der Extended Kalman Filter erweitert das Konzept auf nichtlineare Systeme durch Linearisierung. Und mit robot_localization haben Sie ein produktionsreifes Werkzeug für die Sensorfusion in ROS 2.

+ **Kalman-Filter**: Optimaler Schätzer für lineare Systeme mit Gauß'schem Rauschen

+ **Normalverteilungs-Annahme**: Ermöglicht kompakte Repräsentation durch $\mu$ und $\sigma^2$

+ **Predict-Update-Zyklus**:
  - Predict: Zustand vorhersagen, Unsicherheit wächst
  - Update: Mit Messung korrigieren, Unsicherheit schrumpft

+ **Kalman-Gain**: Balanciert automatisch zwischen Vorhersage und Messung

+ **Extended Kalman Filter**: Erweitert KF auf nichtlineare Systeme durch Linearisierung

+ **robot_localization**: Produktionsreife EKF/UKF-Implementierung für ROS 2

+ **Quality of Service**: QoS-Profile steuern Zuverlässigkeit und Latenz der Sensordaten-Kommunikation

> **Nächste Vorlesung**: Kinematik, Dynamik und Regelungstechnik - Wie steuern wir den Roboter basierend auf unserer Zustandsschätzung?

## Literatur und Ressourcen

+ R. Labbe: [Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python) - Hervorragendes interaktives Tutorial

+ S. Thrun, W. Burgard, D. Fox: *Probabilistic Robotics*, MIT Press, 2005 - Standardwerk für probabilistische Robotik

+ robot_localization Dokumentation: https://docs.ros.org/en/rolling/p/robot_localization/

+ G. Welch, G. Bishop: [An Introduction to the Kalman Filter](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf) - Klassische Einführung
