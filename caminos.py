import numpy as np
import matplotlib.pyplot as plt

div = 50

pxd1 = 0
pxd2 = 1

pyd1 = 0
pyd2 = .8

##pxd = np.array([[pxd1], [pxd2]])
##pyd = np.array([[pyd1], [pyd2]])

##pxd = np.linspace(pxd1, pxd2, div)
##pyd = np.linspace(pyd1, pyd2, div)

# camino de 2 lineas
ponits_x = np.array([0, .86, .86])
ponits_y = np.array([0, .86, 1.72])

px = np.array([])
py = np.array([])

for i in range(len(ponits_x)-1):
    px = np.append(px, np.linspace(ponits_x[i], ponits_x[i+1], div))
    py = np.append(py, np.linspace(ponits_y[i], ponits_y[i+1], div))


pxd = px
pyd = py


plt.plot(pxd, pyd)
plt.show()

