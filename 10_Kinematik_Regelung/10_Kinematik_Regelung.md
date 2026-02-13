<!--

author:   Sebastian Zug & Georg Jäger & Claude.ai
email:    sebastian.zug@informatik.tu-freiberg.de & Georg.Jaeger@informatik.tu-freiberg.de
version:  0.0.2
language: de
comment:  In dieser Vorlesung werden Kinematik mobiler Roboter und Regelungstechnik behandelt.
narrator: Deutsch Female
attribute: thx

import:   https://raw.githubusercontent.com/liascript-templates/plantUML/master/README.md
          https://github.com/liascript/CodeRunner
          https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/config.md
          https://raw.githubusercontent.com/LiaTemplates/Tikz-Jax/main/README.md

attribute: Danke an Andre Dietrich für seinen Kurs "Einführung Regelungstechnik" aus dem Teile übernommen wurden.

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/refs/heads/master/10_Kinematik_Regelung/10_Kinematik_Regelung.md)

# Kinematik und Regelung

<!-- data-type="none" -->
| Parameter            | Kursinformationen                                                                                                       |
| -------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| **Veranstaltung:**   | @config.lecture                                                                                                         |
| **Semester**         | @config.semester                                                                                                        |
| **Hochschule:**      | `Technische Universität Freiberg`                                                                                       |
| **Inhalte:**         | `Kinematik mobiler Roboter, Regelungstechnik, Bahnfolgeregelung`                                                        |
| **Link auf GitHub:** | https://github.com/TUBAF-IfI-LiaScript/VL_SoftwareprojektRobotik/blob/master/10_Kinematik_Regelung/10_Kinematik_Regelung.md |
| **Autoren**          | @author                                                                                                                 |

![](https://media.giphy.com/media/12psn8ymXy3dYs/giphy.gif)

--------------------------------------------------------------------------------

## Einordnung

Nach der Sensorfusion verfügen wir über eine gefilterte Zustandsschätzung des Roboters. Nun müssen wir diese Information nutzen, um den Roboter gezielt zu bewegen.

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
                       | Reaktive Überwachung |   Ausführung  <--- Heute
                       +----------------------+
 Sensordatenerfassung    ^ ^ ^          | | |     Aktuatoroperationen
                         | | |          v v v
                       .-----------------------.
                       | Umgebung              |
                       .-----------------------.                               .
```

Die heutige Vorlesung fokussiert auf die unterste Ebene: Wie setzen wir Bewegungsbefehle in konkrete Radgeschwindigkeiten um, und wie regeln wir diese?

<!--
style="width: 70%; max-width: 720px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii

       +----------+        +----------+
     +-+--------+ |     +--+--------+ |     +----------+     +----------+
+--> |Sensorik  +-+ --> | Filterung +-+ --> | Regelung | --> | Aktorik  | ---+
|    +----------+       +-----------+       +----------+     +----------+    |
|         VL01-07           VL08-09             VL10             VL10        |
|                                                                            |
|                              .---------------------.                       |
+----------------------------- | Umgebung            | <---------------------+
                               .---------------------.
```

--------------------------------------------------------------------------------

## Kinematik mobiler Roboter

Die Kinematik beschreibt die geometrischen Beziehungen zwischen Bewegungsgrößen, ohne die Kräfte zu berücksichtigen, die diese Bewegungen verursachen. Für mobile Roboter ist insbesondere die Frage relevant: **Wie bewegt sich der Roboter, wenn wir bestimmte Radgeschwindigkeiten vorgeben?**

### Differentialantrieb (Differential Drive)

Der Differentialantrieb ist das am weitesten verbreitete Antriebskonzept für mobile Roboter. Der Roboter verfügt über zwei unabhängig angetriebene Räder auf einer gemeinsamen Achse.


```latex  @tikz.picture
% Parameter
\def\Rl{2.0}      % Radius linkes Rad (R - L/2)
\def\Rr{3.5}      % Radius rechtes Rad (R + L/2)
\def\Rm{2.75}     % Radius Mitte (R)
\def\ang{50}      % Winkel Delta theta

% Hintergrund: Tortenstück ausfüllen
\fill[orange!15]
  (0:\Rl) arc[start angle=0, end angle=\ang, radius=\Rl]
  -- (\ang:\Rr) arc[start angle=\ang, end angle=0, radius=\Rr]
  -- cycle;

% ICC Punkt
\fill (0,0) circle (2.5pt);
\node[left=2pt] at (0,0) {ICC};

% Radiale Linien (Start und Ende)
\draw[gray, dashed] (0,0) -- (0:\Rr);
\draw[gray, dashed] (0,0) -- (\ang:\Rr);

% Bogen linkes Rad (innen) - blau
\draw[blue!70!black, line width=2.5pt, -latex]
  (0:\Rl) arc[start angle=0, end angle=\ang, radius=\Rl];
\node[blue!70!black, left=3pt] at ({\ang/2}:\Rl) {$v_L$};

% Bogen rechtes Rad (außen) - rot
\draw[red!70!black, line width=2.5pt, -latex]
  (0:\Rr) arc[start angle=0, end angle=\ang, radius=\Rr];
\node[red!70!black, right=3pt] at ({\ang/2}:\Rr) {$v_R$};

% Mittlerer Bogen (Robotermitte) - gestrichelt grün
\draw[green!50!black, line width=1.5pt, dashed, -latex]
  (0:\Rm) arc[start angle=0, end angle=\ang, radius=\Rm];
\node[green!50!black, above=2pt] at ({\ang/2}:\Rm) {$v$};

% Winkel Delta theta
\draw[-latex, thick] (0:0.7) arc[start angle=0, end angle=\ang, radius=0.7];
\node at ({\ang/2}:1.0) {$\Delta\theta$};

% Räder am Start (als Ellipsen)
\fill[gray!70] (0:\Rl) ellipse (0.12 and 0.35);
\fill[gray!70] (0:\Rr) ellipse (0.12 and 0.35);
\node[below=8pt] at (0:\Rl) {\small Rad L};
\node[below=8pt] at (0:\Rr) {\small Rad R};

% Roboterkörper am Start (Linie zwischen Rädern)
\draw[very thick] (0:\Rl) -- (0:\Rr);

% Beschriftung Radabstand L
\draw[|<->|, thick] ([shift={(0,-0.8)}]0:\Rl) -- ([shift={(0,-0.8)}]0:\Rr)
  node[midway, below=2pt] {$L$};

% Beschriftung der Radien
\node[gray, below] at (0.5,-1.5) {\small $R - \frac{L}{2}$};
\draw[gray, thin, ->] (0.5,-1.4) to[out=90, in=-90] (1.0, -0.2);

\node[gray, below] at (3.0,-1.5) {\small $R + \frac{L}{2}$};
\draw[gray, thin, ->] (3.0,-1.4) to[out=90, in=-90] (2.8, -0.2);

% Fahrtrichtung (am Ende)
\draw[-latex, very thick, black] (\ang:\Rm) -- ++({\ang+90}:0.8)
  node[above] {Fahrtrichtung};
```

Die Grafik zeigt, wie beide Räder denselben Winkel $\Delta\theta$ überstreichen, aber unterschiedliche Bogenlängen zurücklegen. Da Bogenlänge = Radius × Winkel gilt, folgen die Gleichungen direkt aus der Geometrie.

**Vorwärtskinematik:** Von Radgeschwindigkeiten zur Roboterbewegung

Gegeben seien die Geschwindigkeiten des linken Rades $v_L$ und des rechten Rades $v_R$, sowie der Radabstand $L$. Die resultierende Translationsgeschwindigkeit $v$ und Rotationsgeschwindigkeit $\omega$ des Roboters ergeben sich zu:

$$
v = \frac{v_R + v_L}{2}
$$

$$
\omega = \frac{v_R - v_L}{L}
$$

**Inverse Kinematik:** Von Roboterbewegung zu Radgeschwindigkeiten

Umgekehrt können wir aus gewünschter Translations- und Rotationsgeschwindigkeit die Radgeschwindigkeiten berechnen:

$$
v_L = v - \frac{\omega \cdot L}{2}
$$

$$
v_R = v + \frac{\omega \cdot L}{2}
$$

**Kinematisches Modell im Weltkoordinatensystem:**

Die Pose des Roboters $(x, y, \theta)$ ändert sich gemäß:

$$
\begin{pmatrix}
\dot{x}\\
\dot{y}\\
\dot{\theta}
\end{pmatrix}
=
\begin{pmatrix}
\cos\theta & 0 \\
\sin\theta & 0 \\
0 & 1
\end{pmatrix}
\begin{pmatrix}
v\\
\omega
\end{pmatrix}
$$

> **Wichtige Eigenschaft:** Ein Differentialantrieb kann sich nicht seitlich bewegen (nichtholonome Einschränkung). Er kann aber auf der Stelle drehen ($v = 0, \omega \neq 0$).

> Welche Bewegungskonfigurationen sind möglich? 

### Ackermann-Lenkung

Die Ackermann-Lenkung ist das klassische Automobil-Lenkprinzip. Der Roboter verfügt über eine lenkbare Vorderachse und eine angetriebene Hinterachse. Alle Räder folgen konzentrischen Kreisen um einen gemeinsamen Instantaneous Center of Curvature (ICC).

```latex  @tikz.picture
% Ackermann-Lenkung mit 4 Raedern
% L=3.0 (Radstand), B=1.5 (Spurweite), R=6.4 (Wenderadius zur Mitte)

% Spurweite
\def\B{1.5}

% Kurvenboegen fuer alle 4 Raeder
\draw[blue!50, dashed] (-6.4, 0) ++(0:5.65) arc (0:35:5.65);
\draw[blue!50, dashed] (-6.4, 0) ++(0:7.15) arc (0:35:7.15);
\draw[red!50, dashed] (-6.4, 0) ++(27:6.46) arc (27:55:6.46);
\draw[red!50, dashed] (-6.4, 0) ++(21:7.68) arc (21:50:7.68);

% Fahrzeugkoerper (Rechteck)
\draw[thick, blue!30, fill=blue!5] (-0.75, -0.2) rectangle (0.75, 3.2);

% Hinterachse
\draw[very thick] (-0.75, 0) -- (0.75, 0);

% Hinterrad links (nicht gelenkt)
\fill[gray] (-0.75-0.12, -0.35) rectangle (-0.75+0.12, 0.35);

% Hinterrad rechts (nicht gelenkt)
\fill[gray] (0.75-0.12, -0.35) rectangle (0.75+0.12, 0.35);

% Vorderachse (Lenkgestell)
\draw[thick, gray] (-0.75, 3) -- (0.75, 3);

% Vorderrad links (inneres Rad, groesserer Lenkwinkel delta_i = 27 Grad)
\fill[gray, rotate around={27:(-0.75,3)}] (-0.75-0.12, 2.65) rectangle (-0.75+0.12, 3.35);

% Vorderrad rechts (aeusseres Rad, kleinerer Lenkwinkel delta_o = 21 Grad)
\fill[gray, rotate around={21:(0.75,3)}] (0.75-0.12, 2.65) rectangle (0.75+0.12, 3.35);

% Radstand L und Spurweite B
\draw[<->] (1.3, 0) -- (1.3, 3) node[midway, right] {$L$};
\draw[<->] (-0.75, -0.8) -- (0.75, -0.8) node[midway, below] {$B$};

% Lenkwinkel delta_i (innen, groesser)
\draw[dashed, gray] (-0.75, 3) -- (-0.75, 4.0);
\draw[->, thick, red] (-0.75, 3) ++(90:0.6) arc (90:117:0.6);
\node[red] at (-1.3, 3.8) {$\delta_i$};

% Lenkwinkel delta_o (aussen, kleiner)
\draw[dashed, gray] (0.75, 3) -- (0.75, 4.0);
\draw[->, thick, orange] (0.75, 3) ++(90:0.6) arc (90:111:0.6);
\node[orange] at (1.3, 3.7) {$\delta_o$};

% ICC und Verbindungen
\draw[dashed, gray] (-6.4, 0) -- (-0.75, 0);
\draw[dashed, gray] (-6.4, 0) -- (0.75, 0);
\draw[dashed, gray] (-6.4, 0) -- (-0.75, 3);
\draw[dashed, gray] (-6.4, 0) -- (0.75, 3);
\fill (-6.4, 0) circle (3pt);
\node[below] at (-6.4, 0) {ICC};

% Radien beschriften
\node[gray] at (-3.5, -0.3) {$R$};

% === EINSPURMODELL (Bicycle Model) als Ueberlagerung ===
% Mittellinie (Fahrzeugachse)
\draw[very thick, green!50!black, dashed] (0, 0) -- (0, 3);

% Virtuelles Hinterrad (Mitte)
\draw[green!50!black, thick] (-0.2, 0) -- (0.2, 0);
\fill[green!50!black] (0, 0) circle (3pt);

% Virtuelles Vorderrad (Mitte, mit mittlerem Lenkwinkel delta = 24 Grad)
\draw[green!50!black, thick, rotate around={24:(0,3)}] (-0.25, 3) -- (0.25, 3);
\fill[green!50!black] (0, 3) circle (3pt);

% Mittlerer Lenkwinkel delta
\draw[->, very thick, green!50!black] (0, 3) ++(90:0.9) arc (90:114:0.9);
\node[green!50!black] at (0.1, 4.2) {$\delta$};

% Verbindung ICC zum Einspurmodell
\draw[green!50!black, dashed] (-6.4, 0) -- (0, 0);
\draw[green!50!black, dashed] (-6.4, 0) -- (0, 3);

% Legende
\node[green!50!black, right] at (1.8, 1.5) {\small Einspurmodell};
\draw[green!50!black, dashed, thick] (1.6, 1.5) -- (1.75, 1.5);

% Ackermann-Bedingung
\node at (-3.0, -1.5) {$\cot\delta_o - \cot\delta_i = \frac{B}{L}$};
```

**Ackermann-Bedingung:**

$$\cot\delta_o - \cot\delta_i = \frac{B}{L}$$

Die unterschiedlichen Lenkwinkel $\delta_i$ (innen) und $\delta_o$ (außen) sorgen dafür, dass alle Räder schlupffrei auf konzentrischen Kreisen um das ICC rollen.

**Einspurmodell (grün):**

Eine Vereinfachung ersetzt beide Vorderräder durch ein virtuelles Rad in der Mitte mit mittlerem Lenkwinkel $\delta$.

**Wenderadius:**

$$R = \frac{L}{\tan\delta}$$

**Kinematisches Modell (Einspurmodell):**

$$
\begin{pmatrix}
\dot{x}\\
\dot{y}\\
\dot{\theta}
\end{pmatrix}
=
\begin{pmatrix}
\cos\theta \\
\sin\theta \\
\frac{\tan\delta}{L}
\end{pmatrix}
\cdot v
$$

> **Wichtige Eigenschaft:** Im Gegensatz zum Differentialantrieb kann ein Ackermann-Fahrzeug nicht auf der Stelle drehen. Der minimale Wenderadius ist durch den maximalen Lenkwinkel begrenzt.

> Was bedeutet das für die Bewegungsplanung?

### Omnidirektionale Antriebe

Omnidirektionale Roboter können sich in jede Richtung bewegen, ohne sich vorher drehen zu müssen (holonome Bewegung). Dies wird durch spezielle Radkonstruktionen erreicht.

**Mecanum-Räder:**

Mecanum-Räder haben Rollen im 45°-Winkel zur Radachse. Durch gezielte Kombination der vier Radgeschwindigkeiten kann sich der Roboter in alle Richtungen bewegen.

!?[](https://www.youtube.com/watch?v=0k-Ey9bS9lE)

<!-- data-type="none" -->
| Bewegung      | Vorne-Links | Vorne-Rechts | Hinten-Links | Hinten-Rechts |
| ------------- | ----------- | ------------ | ------------ | ------------- |
| Vorwärts      | +           | +            | +            | +             |
| Rückwärts     | -           | -            | -            | -             |
| Links         | -           | +            | +            | -             |
| Rechts        | +           | -            | -            | +             |
| Drehung links | -           | +            | -            | +             |

> **Vorteil:** Maximale Bewegungsfreiheit
> **Nachteil:** Geringere Traktion, Schlupf, höhere Komplexität

## Bewegung direkt messen - Odometrie

Die Odometrie nutzt die Kinematik, um aus den gemessenen Radumdrehungen die Positionsänderung zu berechnen. Bei einem Differentialantrieb:

$$
\Delta s_L = \frac{2\pi r \cdot n_L}{N}
$$

$$
\Delta s_R = \frac{2\pi r \cdot n_R}{N}
$$

wobei $r$ der Radradius, $n$ die Anzahl der Encoder-Ticks und $N$ die Ticks pro Umdrehung sind.

Die Positionsänderung ergibt sich zu:

$$
\Delta s = \frac{\Delta s_R + \Delta s_L}{2}
$$

$$
\Delta \theta = \frac{\Delta s_R - \Delta s_L}{L}
$$

$$
\begin{aligned}
x_{neu} &= x_{alt} + \Delta s \cdot \cos\left(\theta_{alt} + \frac{\Delta \theta}{2}\right) \\
y_{neu} &= y_{alt} + \Delta s \cdot \sin\left(\theta_{alt} + \frac{\Delta \theta}{2}\right) \\
\theta_{neu} &= \theta_{alt} + \Delta \theta
\end{aligned}
$$

> **Problem:** Odometrie akkumuliert Fehler über die Zeit (Drift). Daher ist die Fusion mit absoluten Sensoren (GPS, Landmarken) wichtig - wie in VL08/09 behandelt!

--------------------------------------------------------------------------------

## Regelungstechnik

Nachdem wir wissen, wie sich der Roboter bewegen soll, müssen wir diese Sollbewegung auch erreichen. Hier kommt die Regelungstechnik ins Spiel.

> __Grundidee__ anhand eines mechanischen Regelungssystem: Der Fliehkraftregler von James Watt (18. Jahrhundert) diente dazu, die Geschwindigkeit einer Dampfmaschine bei variierenden Lasten konstant zu halten, indem er die Kraft auf das Dampfventil anpasste.

![](https://upload.wikimedia.org/wikipedia/commons/3/33/WattReg.jpg "Von James Watt. Nach- und weiter bearbeitet von S.Wetzel - Übertragen aus de.wikipedia nach Commons durch Wdwd mithilfe des CommonsHelper., Gemeinfrei, https://commons.wikimedia.org/w/index.php?curid=17924833")<!--style="width: 80%;"-->
--

### Steuerung vs. Regelung

__Definition Steuerung nach DIN 19226__: „Das Steuern, die Steuerung, ist ein Vorgang in einem System, bei dem eine oder mehrere Größen als Eingangsgrößen andere Größen als Ausgangsgrößen aufgrund der dem System eigentümlichen Gesetzmäßigkeiten beeinflussen. Kennzeichen für das Steuern ist der **offene Wirkungsweg**."

```ascii
              Abbildung Eingangsdaten
                  auf Stellbefehle                     System
                         :                               :
                         :                               :
    +----------+    +----------+    +----------+    +----------+
--->| Messein- |--->| Steuer-  |--->| Stell-   |--->| Strecke  |
    | richtung |    | glied    |    | glied    |    |          |
    +----------+    +----------+    +----------+    +----------+
                                         :
                                         :
                                      Aktuator                                                                                           .
```

Anwendungsbeispiel: Heizungsanlage mit zentralem Außentemperaturfühler - eine Störung durch das Öffnen der Tür kann nicht ausgeglichen werden.

__Definition Regelung nach DIN 19226__: „Das Regeln, die Regelung, ist ein Vorgang, bei dem eine Größe, die zu regelnde Größe (Regelgröße), fortlaufend erfasst, mit einer anderen Größe, der Führungsgröße, verglichen und abhängig vom Ergebnis dieses Vergleichs im Sinne einer Angleichung an die Führungsgröße beeinflusst wird. Kennzeichen für das Regeln ist der **geschlossene Wirkungskreislauf**."

<!--
style="width: 80%; min-width: 420px; max-width: 720px;"
-->
```ascii
    +----------+    +----------+    +----------+    +----------+
--->| Ver-     |--->| Regel-   |--->| Stell-   |--->| Strecke  |----+
 +->| gleicher |    | glied    |    | glied    |    |          |    |
 |  +----------+    +----------+    +----------+    +----------+    |
 |                                                                  |
 |                      +-----------+                               |
 +----------------------| Messein-  |<------------------------------+
                        | richtung  |
                        +-----------+                                                     .
```

| Begriff | Bedeutung |
| ------- | --------- |
| Regelungsgröße $x$ | Istwert - das "Ziel" der Regelung |
| Führungsgröße $w$ | Sollwert - der gewünschte Wert |
| Rückführgröße $r$ | Gemessener Istwert |
| Regeldifferenz $e$ | $e = w - r$ (Soll-Ist-Abweichung) |
| Stellgröße $y$ | Ausgabe des Reglers an den Aktuator |
| Störgröße $z$ | Unerwünschte äußere Einflüsse |

### Folgeregler vs. Festwertregler

<!-- data-type="none" -->
| Merkmal     | Folgeregler                                                                      | Festwertregler                                                                   |
| ----------- | -------------------------------------------------------------------------------- | -------------------------------------------------------------------------------- |
| Führungsgröße | variabel                                                                       | konstant                                                                         |
| Performance | Gutes Führungsverhalten erforderlich (schnelles Folgen der Sollwertvorgabe)     | Gutes Störverhalten erforderlich (schnelles Ausregeln von Störungen)             |
| Beispiel    | Abstandsbasierte Geschwindigkeitsregelung eines Fahrzeuges                       | Inverses Pendel, Temperaturregelung                                              |


### Streckenverhalten

Die Strecke (das zu regelnde System) reagiert nicht unmittelbar auf Reglerinputs, sondern zeigt ein eigenes Verzögerungsverhalten. Die folgenden Diagramme zeigen jeweils die Sprungantwort typischer Streckentypen.

**Proportionale Strecke (P-Glied)**

![P-Glied](./images/Step-p.png)

$x(t) = K_p \cdot y(t)$ mit $K_p = 2$

Beispiele: Verstärker, Spannungsteiler, Sensoren mit vernachlässigbarem Verzögerungsverhalten

**Integral wirkende Strecke (I-Glied)**

![I-Glied](./images/Step-i.png)

$\dot{y}(t) = K_i \cdot u(t)$ mit $K_i = 2$

Beispiele: Wassertank, Kondensator, Geschwindigkeit als Integration der Beschleunigung

**Verzögerungsglied 0. Ordnung (Totzeitglied)**

![Totzeitglied](./images/Step-t0.png)

$y(t) = u(t-T_t)$ mit $T_t = 0.5s$

Beispiele: Rechenzeit für die Sensordatenverarbeitung, Material auf einem Förderband

**Verzögerungsglied 1. Ordnung (PT1-Glied)**

![PT1-Glied](./images/Step_PT1.png)

$y(t) = K(1-e^{-\frac{t}{T}})$ mit $K = 2, T = 1s$

Beispiele: Gleichstrommotor (Spannung → Drehzahl)

**Verzögerungsglied 2. Ordnung (PT2-Glied)**

![PT2-Glied](./images/Step_PT2.png)

$a_2 \ddot{y}(t) + a_1 \dot{y}(t) + a_0 y(t) = b_0 u(t)$ mit $K = 2, T = 1, D = \{0.2, 1, 5\}$ (D = Dämpfung)

Beispiele: Zwei hintereinander geschaltete PT1-Glieder, Gleichstrommotor mit berücksichtigter Induktivität

### Der PID-Regler

Der PID-Regler kombiniert drei Regelungsstrategien:

**Proportionalanteil (P):** Reagiert proportional auf den aktuellen Fehler

$$ u_P(t) = K_P \cdot e(t) $$

- Schnelle Reaktion
- Bleibende Regelabweichung möglich

**Integralanteil (I):** Integriert den Fehler über die Zeit

$$ u_I(t) = K_I \cdot \int_0^t e(\tau) d\tau $$

- Eliminiert bleibende Regelabweichung
- Langsamer, kann zu Überschwingen führen

**Differentialanteil (D):** Reagiert auf die Änderungsrate des Fehlers

$$ u_D(t) = K_D \cdot \frac{de(t)}{dt} $$

- Dämpft Schwingungen
- Empfindlich gegenüber Messrauschen

**Kombinierter PID-Regler:**

$$ u(t) = K_P \cdot e(t) + K_I \cdot \int_0^t e(\tau) d\tau + K_D \cdot \frac{de(t)}{dt} $$

Diskretisiert für die Implementierung:

$$ u[k] = K_P \cdot e[k] + K_I \cdot \sum_{i=0}^{k} e[i] \cdot \Delta t + K_D \cdot \frac{e[k] - e[k-1]}{\Delta t} $$

**Auswirkung der Parameter:**

![PID Animation](./images/PID_Compensation_Animated.gif "_Effects of varying PID parameters (Kp,Ki,Kd) on the step response of a system - Wikimedia, Autor: Physicsch_")

<!-- data-type="none" -->
| Parameter | Zu klein                                 | Zu groß                                       |
| --------- | ---------------------------------------- | --------------------------------------------- |
| $K_P$     | Langsame Reaktion, große Regelabweichung | Überschwingen, Instabilität                   |
| $K_I$     | Bleibende Regelabweichung                | Starkes Überschwingen, langsames Einschwingen |
| $K_D$     | Wenig Dämpfung                           | Verstärkung von Rauschen                      |

### Reglerperformanz und Gütekriterien

Die Qualität eines Reglers lässt sich anhand verschiedener Kriterien bewerten:

**Güteforderungen:**

- **Stabilität:** Der Regelkreis muss stabil sein - Abweichungen dürfen nicht unbegrenzt wachsen
- **Störkompensation:** Wie gut werden externe Störungen ausgeregelt?
- **Sollwertfolge:** Wie schnell und genau folgt die Regelgröße der Führungsgröße?
- **Robustheit:** Bleibt der Regler auch bei Parameteränderungen der Strecke stabil?

**Grenzen des PID-Reglers:**

Der PID-Regler deckt etwa 90% der praktischen Anwendungsfälle ab, stößt aber an Grenzen bei:

- Nicht-linearen Prozessen
- Systemen mit großer Totzeit
- Fehlendem realen Leistungsverhalten der Aktorik

### Implementierungsbeispiel

```python PID_controller.py
import numpy as np
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, Kp, Ki, Kd, dt=0.01):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0
        self.prev_error = 0

    def update(self, setpoint, measurement):
        error = setpoint - measurement

        # Proportional
        P = self.Kp * error

        # Integral
        self.integral += error * self.dt
        I = self.Ki * self.integral

        # Derivative
        derivative = (error - self.prev_error) / self.dt
        D = self.Kd * derivative

        self.prev_error = error

        return P + I + D

# Simulation eines PT1-Systems (z.B. Motor)
def simulate_pt1(u, y_prev, K=1.0, T=0.5, dt=0.01):
    # PT1: T * dy/dt + y = K * u
    dy = (K * u - y_prev) / T
    return y_prev + dy * dt

# Simulation
dt = 0.01
t_end = 5.0
t = np.arange(0, t_end, dt)
setpoint = 1.0

# Verschiedene PID-Einstellungen testen
configs = [
    {"Kp": 1.0, "Ki": 0.0, "Kd": 0.0, "label": "P only"},
    {"Kp": 1.0, "Ki": 0.5, "Kd": 0.0, "label": "PI"},
    {"Kp": 2.0, "Ki": 1.0, "Kd": 0.2, "label": "PID"},
]

plt.figure(figsize=(10, 6))
for config in configs:
    pid = PIDController(config["Kp"], config["Ki"], config["Kd"], dt)
    y = np.zeros(len(t))

    for i in range(1, len(t)):
        u = pid.update(setpoint, y[i-1])
        y[i] = simulate_pt1(u, y[i-1], dt=dt)

    plt.plot(t, y, label=config["label"])

plt.axhline(y=setpoint, color='k', linestyle='--', label='Setpoint')
plt.xlabel('Zeit [s]')
plt.ylabel('Regelgröße')
plt.title('Vergleich verschiedener Reglereinstellungen')
plt.legend()
plt.grid(True)
plt.savefig('pid_comparison.png')
```
@LIA.eval(`["main.py"]`, `none`, `python3 main.py`)

--------------------------------------------------------------------------------

## Bahnfolgeregelung

Geschwindigkeits- oder Lageregelungen sind nur ein Teilaspekt der Robotersteuerung. 

> Die Bahnfolgeregelung (Trajectory Tracking) ist die Aufgabe, einen Roboter entlang einer vorgegebenen Bahn zu führen. Wir betrachten zwei klassische Algorithmen.

### Move-to-Pose Controller

> Das folgende Beispiel ist dem Buch von Peter Corke, "Robotics, Vision & Control", Springer 2017 entnommen. Der [Code](https://github.com/AtsushiSakai/PythonRobotics/tree/master/Control/move_to_pose) basiert auf einer Implementierung von AtsushiSakai.

Wir wollen einen Roboter von einer Startpose zu einer Zielpose fahren lassen. Ausgangspunkt ist das kinematische Modell eines differentiell getriebenen Roboters:

$$
\begin{pmatrix}
\dot{x}\\
\dot{y}\\
\dot{\theta}
\end{pmatrix}
=
\begin{pmatrix}
\cos\theta & 0 \\
\sin\theta & 0 \\
0 & 1
\end{pmatrix}
\begin{pmatrix}
v\\
\omega
\end{pmatrix}
$$

![Kinematisches Modell](./images/robot_kinematic_model.png "_Kinematisches Modell eines Differentialantriebs_")

Der Fehler zwischen der aktuellen und der Zielposition lässt sich in Polarkoordinaten darstellen:

$$
\begin{aligned}
\rho &= \sqrt{\Delta_x^2 + \Delta_y^2} \\
\alpha &= \arctan\frac{\Delta_y}{\Delta_x} - \theta \\
\beta &= -\theta - \alpha
\end{aligned}
$$

Dabei ist $\rho$ der Abstand zum Ziel, $\alpha$ der Winkel zwischen Roboterausrichtung und Zielrichtung, und $\beta$ kodiert die gewünschte Endorientierung.

Die Bewegungsgleichung in Polarkoordinaten (für $-\frac{\pi}{2} \leq \alpha < \frac{\pi}{2}$):

$$
\begin{pmatrix}
\dot{\rho}\\
\dot{\alpha}\\
\dot{\beta}
\end{pmatrix}
=
\begin{pmatrix}
-\cos\alpha & 0 \\
\frac{\sin\alpha}{\rho} & -1 \\
-\frac{\sin\alpha}{\rho} & 0
\end{pmatrix}
\begin{pmatrix}
v\\
\omega
\end{pmatrix}
$$

Da $\rho$, $\alpha$ und $\beta$ den Fehler gegenüber der Zielgröße darstellen, können wir diese mit proportionaler Verstärkung auf die Bewegungsvorgaben abbilden:

$$
\begin{aligned}
v &= k_\rho \cdot \rho \\
\omega &= k_\alpha \cdot \alpha + k_\beta \cdot \beta
\end{aligned}
$$

| Parameter | Wirkung |
|-----------|---------|
| $k_\rho$ | Geschwindigkeit der Annäherung |
| $k_\alpha$ | Ausrichtung auf das Ziel |
| $k_\beta$ | Ausrichtung der Endpose |

![Move to Pose](./images/move_to_pose.png "_Trajektorien mit verschiedenen Reglerparametern: rot ($k_\rho=5, k_\alpha=3, k_\beta=3$), blau ($k_\rho=5, k_\alpha=15, k_\beta=3$)_")

> **Aufgabe:** Erklären Sie den Einfluss der drei Reglerparameter auf die Bewegung des Roboters. Welchen Einfluss haben Beschränkungen der maximalen Linear- und Winkelgeschwindigkeit?

### Pure Pursuit Controller

Der Pure Pursuit Controller ist ein geometrischer Ansatz, der einen Punkt auf der Sollbahn "verfolgt".

``` latex @tikz
\begin{tikzpicture}[scale=1.2]
% Sollbahn (horizontale Linie)
\draw[very thick, blue!60!black] (-2, 0) -- (6, 0);
\node[blue!60!black, below] at (5.5, -0.1) {Sollbahn};

% Lookahead-Kreis um den Roboter (Radius Ld = 2.915)
\draw[thin, purple, dashed] (1.5, -1.5) circle (2.915);
\node[purple, below left] at (-0.8, -2.8) {Lookahead-Kreis};

% Roboter Position und Körper
\draw[fill=gray!20, thick] (1.1, -1.75) rectangle (1.9, -1.25);
% Räder
\fill[black] (1.15, -1.35) rectangle (1.25, -1.65);
\fill[black] (1.75, -1.35) rectangle (1.85, -1.65);
% Roboter-Label
\node[below] at (1.5, -1.85) {Roboter};

% Verworfener Schnittpunkt (hinter dem Roboter)
\fill[gray!50] (-1, 0) circle (4pt);
\node[gray!50, above left] at (-1, 0) {\small verworfen};
\draw[thin, gray, dotted] (1.5, -1.5) -- (-1, 0);

% Gewählter Lookahead-Punkt (vor dem Roboter)
\fill[red] (4, 0) circle (4pt);
\node[red, above right] at (4, 0) {Lookahead-Punkt};

% Verbindung Roboter zum Lookahead-Punkt (Ld)
\draw[thick, green!50!black, dashed] (1.5, -1.5) -- (4, 0);
\node[green!50!black, rotate=31] at (2.5, -0.55) {$L_d$};

% Vorwärtsrichtung des Roboters
\draw[thick, ->, gray] (1.5, -1.5) -- (1.5, -0.3);
\node[gray, left] at (1.5, -0.3) {Fahrtrichtung};

% Winkel alpha
\draw[thick, red, ->] (1.5, -1.5) ++(90:0.5) arc (90:31:0.5);
\node[red] at (2.0, -0.9) {$\alpha$};

% Projektion auf die Bahn
\draw[thin, dotted, gray] (1.5, -1.5) -- (1.5, 0);
\fill[gray] (1.5, 0) circle (2pt);

% Kreisbogen (Pfad zum Lookahead-Punkt)
% Mittelpunkt bei (3.2, -1.5), Radius 1.7
\draw[orange, thick, dashed] (1.5, -1.5) arc (180:62:1.7);
\node[orange, below] at (2.8, -1.7) {Fahrtkreis};

% Kreismittelpunkt (ICC)
\fill[orange] (3.2, -1.5) circle (2pt);
\node[orange, right] at (3.3, -1.5) {\small ICC};

% Radius R
\draw[thin, orange, dotted] (3.2, -1.5) -- (1.5, -1.5);
\node[orange, above] at (2.35, -1.5) {\small $R$};
\end{tikzpicture}
```

**Algorithmus:**

1. Finde den Punkt auf der Bahn, der die Lookahead-Distanz $L_d$ vom Roboter entfernt ist
2. Berechne den Winkel $\alpha$ zum Lookahead-Punkt
3. Berechne die Krümmung, die den Roboter zum Punkt führt

**Mehrere Schnittpunkte:** Der Lookahead-Kreis kann die Trajektorie an mehreren Punkten schneiden. In diesem Fall wird der Punkt gewählt, der:

- am weitesten in Fahrtrichtung auf der Bahn liegt, oder
- den größten Bahnfortschritt (Pfadindex) hat

Punkte "hinter" dem Roboter (bezogen auf die Bahnrichtung) werden verworfen.

Die Lenkrate ergibt sich zu:

$$
\kappa = \frac{2 \sin(\alpha)}{L_d}
$$

Für einen Differentialantrieb ergibt sich die Winkelgeschwindigkeit:

$$
\omega = v \cdot \kappa = \frac{2 \, v \, \sin(\alpha)}{L_d}
$$

Die Radgeschwindigkeiten (bei Spurweite $b$):

$$
v_r = v + \frac{\omega \cdot b}{2}, \quad v_l = v - \frac{\omega \cdot b}{2}
$$

**Lookahead-Distanz:** Die Wahl von $L_d$ beeinflusst das Verhalten:
- Kleines $L_d$: Genauere Bahnverfolgung, aber oszillierendes Verhalten
- Großes $L_d$: Glattere Bewegung, aber "Abkürzungen" in Kurven

Häufig wird $L_d$ (wie bei ROS2) die Prüfdistanz $L_d$ geschwindigkeitsabhängig gewählt:

$$
L_d = k \cdot v + L_{d,min}
$$

> Eine sehr umfangreiche Analyse des Pure Pursuit Algorithmus findet sich [Basic Pure Pursuit](https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit)

### Vergleich der Ansätze

<!-- data-type="none" -->
| Kriterium | Pure Pursuit | Move-to-Pose |
|-----------|--------------|--------------|
| **Eingabe** | Trajektorie (Wegpunktliste) | Nur Zielpose |
| **Pfad** | Vorgegeben (z.B. von Pfadplaner) | Vom Regler selbst bestimmt |
| **Regelgröße** | Lookahead-Punkt | Polarkoordinaten-Fehler |
| **Endorientierung** | Nicht geregelt | Explizit geregelt ($k_\beta$) |
| **Anwendung** | Bahnverfolgung | Punktnavigation, Andocken |

**Wann welchen Ansatz?**

- **Move-to-Pose**: Einfache Punkt-zu-Punkt Navigation ohne Hindernisse
- **Pure Pursuit**: Wenn ein Pfadplaner die Trajektorie vorgibt

> **Ausblick:** In der nächsten Vorlesung behandeln wir, wie diese Trajektorien entstehen: Kartenrepräsentationen, Pfadplanungsalgorithmen (A*, RRT) und den ROS2 Navigation Stack.

--------------------------------------------------------------------------------

## ROS2 Integration

### Twist-Messages

In ROS2 werden Bewegungsbefehle typischerweise als `geometry_msgs/Twist` übertragen:

```
# geometry_msgs/Twist
Vector3 linear    # Lineare Geschwindigkeit (x, y, z)
Vector3 angular   # Winkelgeschwindigkeit (x, y, z)
```

Für einen Differentialantrieb sind relevant:
- `linear.x` = Vorwärtsgeschwindigkeit $v$
- `angular.z` = Drehrate $\omega$

### ros2_control

Das `ros2_control` Framework bietet eine standardisierte Schnittstelle zwischen Controllern und Hardware:

```ascii
+----------------+     +------------------+     +------------------+
| Controller     | --> | Controller       | --> | Hardware         |
| Manager        |     | (z.B. diff_drive)|     | Interface        |
+----------------+     +------------------+     +------------------+
        |                      |                        |
        v                      v                        v
   Controller            cmd_vel              Motoransteuerung
   Konfiguration       /joint_states                                                                                                  .
```

**Diff-Drive-Controller:** Konvertiert `cmd_vel` (Twist) in Radgeschwindigkeiten:

```yaml
# diff_drive_controller.yaml
diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.34
    wheel_radius: 0.05
    publish_rate: 50.0
```

### Nav2 Controller Server

Der Navigation Stack (Nav2) verwendet den Controller Server für die Bahnverfolgung:

```ascii
+-------------+      +------------------+      +------------------+
| Planner     | ---> | Controller       | ---> | cmd_vel          |
| (Global)    | Path | Server           | Twist| (Roboter)        |
+-------------+      +------------------+      +------------------+
                            |
                            | nutzt
                            v
                     +------------------+
                     | Controller       |
                     | Plugin           |
                     | (z.B. DWB, TEB)  |
                     +------------------+                                                                 .
```

**Verfügbare Controller-Plugins:**

<!-- data-type="none" -->
| Plugin | Beschreibung |
|--------|--------------|
| DWB (Dynamic Window Approach) | Sampling-basiert, berücksichtigt Hindernisse |
| TEB (Timed Elastic Band) | Optimierungsbasiert, zeitoptimal |
| Regulated Pure Pursuit | Pure Pursuit mit Geschwindigkeitsregulierung |
| MPPI | Model Predictive Path Integral |

**Beispielkonfiguration für Regulated Pure Pursuit:**

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.8
      use_velocity_scaled_lookahead_dist: true
```

--------------------------------------------------------------------------------

## Zusammenfassung

In dieser Vorlesung haben wir behandelt:

1. **Kinematik mobiler Roboter**
   - Differentialantrieb: Einfach, kann auf der Stelle drehen
   - Ackermann-Lenkung: Wie Autos, minimaler Wenderadius
   - Omnidirektionale Antriebe: Maximale Bewegungsfreiheit

2. **Regelungstechnik**
   - Steuerung (offener Wirkungsweg) vs. Regelung (geschlossener Kreis)
   - PID-Regler: P (schnell), I (keine bleibende Abweichung), D (Dämpfung)

3. **Bahnfolgeregelung**
   - Pure Pursuit: Geometrischer Ansatz mit Lookahead
   - Stanley Controller: Kombiniert Heading- und Cross-Track-Error

4. **ROS2 Integration**
   - `geometry_msgs/Twist` für Bewegungsbefehle
   - `ros2_control` für Hardware-Abstraktion
   - Nav2 Controller Server für Navigation

> **Nächste Vorlesung:** Mit Navigation (SLAM, Pfadplanung) schließen wir den Kreis und können autonome Roboter realisieren!

--------------------------------------------------------------------------------

