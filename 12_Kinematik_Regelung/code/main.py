# https://www.csestack.org/control-systems-simulation-python-example/
import math
import matplotlib.pyplot as plt
import numpy as np

steps = 2000

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

fig, ax = plt.subplots()

Plant = SecondOrderSystem(250, 100)     
t = np.arange(0, steps)
y = np.arange(0, steps, dtype = float)
for speed in [10, 100, 150, 200]:
  for index, value in enumerate(t):
      y[index]=Plant(speed)
  ax.plot(t, y, label = f"Voltage level '{speed}'")
  Plant.init_system()

plt.xlabel("Time")
plt.ylabel("Speed")
plt.legend()
plt.grid()  
#plt.show()
plt.savefig('foo.png')
