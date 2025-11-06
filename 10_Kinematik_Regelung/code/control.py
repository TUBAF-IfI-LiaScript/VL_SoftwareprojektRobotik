# https://www.csestack.org/control-systems-simulation-python-example/
import math
import matplotlib.pyplot as plt
import numpy as np

steps = 1000

class SecondOrderSystem:
  def __init__(self, d1, d2):
    if d1 > 0:
      e1 = -1.0 / d1
      x1 = math.exp(e1)
    else: x1 = 0
    if d2 > 0:
      e2 = -1.0 / d2
      x2 = math.exp(e2)
    else: x2 = 0
    a = 1.0 - x1    # b = x1
    c = 1.0 - x2    # d = x2
    self.ac = a * c
    self.bpd = x1 + x2
    self.bd = x1 * x2
    self.init_system()

  def init_system(self):
    self.Yppr = 0
    self.Ypr = 0

  def __call__(self, X):
    Y = self.ac * X + self.bpd * self.Ypr - self.bd * self.Yppr
    self.Yppr = self.Ypr
    self.Ypr = Y
    return Y

class PIDControlBlock:
  def __init__(self, Kp, Ki, Kd):
    self.Kp = Kp
    self.Ki = Ki
    self.Kd = Kd
    self.Epr = 0
    self.Eppr = 0
    self.Epppr = 0
    self.Sum = 0

  def __call__(self, E):
    self.Sum += 0.5 * self.Ki * (E + self.Epr)      # where T ~1
    U = self.Kp * E + self.Sum + 0.1667 * self.Kd * (E - self.Epppr + 3.0 * (self.Epr - self.Eppr))
    self.Epppr = self.Eppr
    self.Eppr = self.Epr
    self.Epr = E
    return U

class ClosedLoopSystem:
  def __init__(self, controller, plant) :
    self.P = plant
    self.C = controller
    self.Ypr = 0

  def __call__(self, X):
    E = X - self.Ypr
    U = self.C(E)
    Y = self.P(U)
    self.Ypr = Y
    return Y

Plant = SecondOrderSystem(250, 100)     
#Pid = PIDControlBlock(5, 0.0143, 356.25)
Pid = PIDControlBlock(2, 0, 0)
Ctrl = ClosedLoopSystem(Pid, Plant)
t = np.arange(0, 1001)
setpoints = np.zeros(len(t))
setpoints[np.where(t > 50)]= 100
y = np.arange(0, 1001, dtype = float)
for index, value in enumerate(setpoints):
    y[index]=Ctrl(value)
fig, ax = plt.subplots()
ax.plot(t, setpoints, label = "Setpoint")
ax.plot(t, y, label = "Controled speed level")
plt.xlabel("Time")
plt.ylabel("Speed")
plt.legend()
plt.grid()
plt.show()
#plt.savefig('foo.png')
