import socket
from matplotlib import pyplot as plt
import numpy as np
import time
import pandas as pd

# Parámetros de simulación
tf = 30  # tiempo de simulación
ts = 0.05  # tiempo de muestreo
t = np.arange(0, tf+ts, ts)  # vector de tiempo

N = len(t)  # longitud del vector de tiempo

############# Parámetros del robot #############
a = 0.105  # distancia del eje de las ruedas al centro del robot (vertical)
b = 0.105  # distancia del eje de las ruedas al centro del robot (horizontal)
r = 0.04   # radio de las ruedas

############## Condiciones iniciales ############
hx = np.zeros(N+1)  # vector de posición en x
hy = np.zeros(N+1)  # vector de posición en y
theta = np.zeros(N+1)  # vector de orientación inicial

hx[0] = 0  # posición en x
hy[0] = 0  # posición en y
theta[0] = 0*(np.pi/180)  # orientación inicial

############## Camino a seguir #############
vMax = 0.3  # velocidad máxima del robot
div = 3000
# div = 4000
# camino de 2 lineas
ponits_x = [0, 0, 2.15, 2.15]  # puntos y
ponits_y = [0, -0.86, -0.86, 0.86]  # puntos x
# ponits_x = [0, 0]  # puntos x
# ponits_y = [0, -2.06]  # puntos y

px = []
py = []

for i in range(len(ponits_x)-1):
    px.append(np.linspace(ponits_x[i], ponits_x[i+1], div))
    py.append(np.linspace(ponits_y[i], ponits_y[i+1], div))

pxd = np.hstack(px)
pyd = np.hstack(py)

sizePoints = len(pxd)

beta = np.zeros(sizePoints)  # angulo beta

for i in range(sizePoints):
    if i == 0:
        beta[i] = np.arctan2(pyd[i+1]-pyd[i], pxd[i+1]-pxd[i])
    else:
        beta[i] = np.arctan2(pyd[i]-pyd[i-1], pxd[i]-pxd[i-1])

thetad = -2*(np.pi/180)*np.ones(N)  # orientación deseada

thetad_dot = np.zeros(N)  # velocidad angular deseada

############## Velocidades de referencia ############
vf_ref = np.zeros(N)  # velocidad frontal en m/s
vl_ref = np.zeros(N)  # velocidad lateral en m/s
w_ref = np.zeros(N)  # velocidad angular en rad/s

############# Velocidades medidas ############
vf_med = np.zeros(N)  # velocidad frontal medida en m/s
vl_med = np.zeros(N)  # velocidad lateral medida en m/s
w_med = np.zeros(N)  # velocidad angular medida en rad/s

############ Errores ############
hxe = np.zeros(N)  # error en x
hye = np.zeros(N)  # error en y
thetae = np.zeros(N)  # error en orientación

########## Comunicación por socket ##########
HOST = '192.168.4.1'  # Dirección IP del ESP32 configurado como AP
PORT = 80             # Puerto del servidor ESP32

######## Variables para guardar datos ########
data = {"vf_med": 0, "vl_med": 0, "w_med": 0}
df = pd.DataFrame(data, index=[0])

list_vf_med = []
list_vl_med = []
list_w_med = []
list_vf_ref = []
list_vl_ref = []
list_w_ref = []
vf = 0
vl = 0
w = 0


def send_data(sock: socket.socket, data: str):
    data = data + "\n"
    sock.sendall(data.encode())


def get_data(sock) -> list:
    global df, vf, vl, w
    data_recived = sock.recv(1024).decode().strip("\r\n")
    # print(data_recived)
    values = data_recived.split(",")

    if len(values) == 6 and values[0] == "vf" and values[2] == "vl" and values[4] == "w":
        vf = float(values[1])
        vl = float(values[3])
        w = float(values[5])

    list_vf_med.append(vf)
    list_vl_med.append(vl)
    list_w_med.append(w)
    new_row = pd.Series([vf, vl, w], index=['vf_med', 'vl_med', 'w_med'])
    df = pd.concat([df, new_row.to_frame().T], ignore_index=True)
    return [vf, vl, w]


# Crear el socket TCP/IP
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
time.sleep(1)
### Definir punto de parada con ts-time_slapsed
##if len(ponits_x)>3:
##    stop_index = sizePoints - 1225
##else:
##    stop_index = sizePoints - 210

# Definir punto de parada con 
# if len(ponits_x) > 3:
#     stop_index = sizePoints - 1190
# else:
stop_index = sizePoints - 100



for k in range(N):
    start_time = time.time()
    # punto mas cercano
    min_dist = 1.5
    for i in range(sizePoints):
        dist = np.sqrt((pxd[i]-hx[k])**2 + (pyd[i]-hy[k])**2)
        if dist < min_dist:
            min_dist = dist
            index = i
    # Verificar si el robot está en la última sección del camino y ha alcanzado el índice de parada
    if index >= stop_index:            
        break
    # Error
    hxe[k] = pxd[index] - hx[k]
    hye[k] = pyd[index] - hy[k]
    thetae[k] = thetad[k] - theta[k]
    # Vector columna de errores 3x1
    he = np.array([[hxe[k]], [hye[k]], [thetae[k]]])
    # Matriz de Jacobiana
    J = np.array([[np.cos(theta[k]), -np.sin(theta[k]), 0],
                  [np.sin(theta[k]), np.cos(theta[k]), 0],
                  [0, 0, 1]])
    # Parámetros de control
    ##K = 0.82*np.array([[1.2, 0, 0],  con ts-time_slapsed
    ##                   [0, 1.8, 0],
    ##                   [0, 0, 1.5]])
    ##K = 0.82*np.array([[1.2, 0, 0], a Vmax = 0.26
    ##                   [0, 1.35, 0],
    ##                   [0, 0, 1.3]])
    K = 0.82*np.array([[3.2, 0, 0],
                       [0, 2.9, 0],
                       [0, 0, 1.39]])
    # Velocidades de referencia
    pxdp = vMax * np.cos(beta[index])
    pydp = vMax * np.sin(beta[index])
    # Velocidades de referencia en coordenadas generalizadas (3x1)
    pdp = np.array([[pxdp], [pydp], [thetad_dot[k]]])
    # Ley de control
    # velocidad de referencia en coordenadas generalizadas (3x1)
    qp_ref = np.linalg.pinv(J) @ (pdp + K@he)
    # Aplicar acciones de control
    vf_ref[k] = qp_ref[0][0]
    vl_ref[k] = qp_ref[1][0]
    w_ref[k] = qp_ref[2][0]
    # Enviar datos al robot
    send_data(s, f"{vf_ref[k]},{vl_ref[k]},{w_ref[k]}")
    vf_med[k], vl_med[k], w_med[k] = get_data(s)
    # Integral numérica para calcular la orientación
    theta[k + 1] = theta[k] + ts * w_med[k]
    # Modelo cinemático
    hxp = vf_med[k] * np.cos(theta[k + 1]) - \
        vl_med[k] * np.sin(theta[k + 1])
    hyp = vf_med[k] * np.sin(theta[k + 1]) + \
        vl_med[k] * np.cos(theta[k + 1])
    hx[k + 1] = hx[k] + ts * hxp
    hy[k + 1] = hy[k] + ts * hyp
    # Tiempo de muestreo
    time_elapsed = time.time() - start_time
    time.sleep( time_elapsed)

for i in range(20):
    send_data(s, "0,0,0")
    time.sleep(0.05)

s.close()
df.to_excel("datos_controlador.xlsx")

plt.figure()
plt.plot(t, vf_ref, label='vf_ref')
plt.plot(t, vf_med, label='vf_med')
plt.xlabel('Tiempo [s]')
plt.ylabel('Velocidad Frontal [m/s]')
plt.legend()
plt.grid()

plt.figure()
plt.plot(t, vl_ref, label='vl_ref')
plt.plot(t, vl_med, label='vl_med')
plt.xlabel('Tiempo [s]')
plt.ylabel('Velocidad Lateral [m/s]')
plt.legend()
plt.grid()

plt.figure()
plt.plot(t, w_ref, label='w_ref')
plt.plot(t, w_med, label='w_med')
plt.xlabel('Tiempo [s]')
plt.ylabel('Velocidad Angular [rad/s]')
plt.legend()
plt.grid()

plt.figure()
plt.plot(hy, hx, label='Trayectoria Real')
plt.plot(pyd, pxd, '--', label='Trayectoria Deseada')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.legend()
plt.grid()

plt.figure()
plt.plot(t, hxe, label='Error en x')
plt.plot(t, hye, label='Error en y')
plt.xlabel('Tiempo [s]')
plt.ylabel('Error en y [m]')
plt.legend()
plt.grid()

plt.figure()
plt.plot(t, thetae, label='Error en orientación')
plt.xlabel('Tiempo [s]')
plt.ylabel('Error en orientación [rad]')
plt.legend()
plt.grid()

plt.show()
