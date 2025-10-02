import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt

# ==================== Robo ====================

# Declaracao do manipulador
l0, l1, l2, l3, l4 = 1, 1, 1, 1, 1

# Criação dos links com massa e inércia unitária
link1 = rtb.RevoluteDH(d=(l0 + l2), a=l1, m=1.0, r=[0,0,0], I=np.eye(3))
link2 = rtb.RevoluteDH(a=l3, m=1.0, r=[0,0,0], I=np.eye(3))
link3 = rtb.PrismaticDH(offset=-l4, alpha=-np.pi, m=1.0, r=[0,0,0], I=np.eye(3))

# Criação do robô
robot = rtb.DHRobot([link1, link2, link3])

# ================== Trajetória ==================

# Posicoes das juntas
q_start = np.array([0, 0, 0])
q_end = np.array([np.pi/4, np.pi/3, -0.5])

# Tempo de simulacao
time_s = 5.0
dt = 0.05
steps = int(time_s / dt)
t = np.linspace(0, time_s, steps)

# Polinômio de 5º grau (quintic)
traj = np.zeros((steps, 3))
qd = np.zeros((steps, 3))
qdd = np.zeros((steps, 3))

# Gerando a trajetoria
for i, t_i in enumerate(t):
    # porcentagem de conclusao da trajetoria
    s = t_i / time_s
    # posicoes das juntas
    traj[i, :] = q_start + (q_end - q_start)*(10*s**3 - 15*s**4 + 6*s**5)
    # velocidade
    qd[i, :]   = (q_end - q_start)*(30*s**2 - 60*s**3 + 30*s**4)/time_s
    # aceleracao
    qdd[i, :]  = (q_end - q_start)*(60*s - 180*s**2 + 120*s**3)/(time_s**2)

# ================== Calculando torques ==================
tau = np.zeros((steps, 3))
for i in range(steps):
    tau[i, :] = robot.rne(traj[i, :], qd[i, :], qdd[i, :])

# ================== Plotando ==================
juntas = ['J1', 'J2', 'J3']

# Posição, velocidade e aceleração
fig, axs = plt.subplots(3, 1, figsize=(8, 10))
for i in range(3):
    axs[0].plot(t, traj[:, i], label=juntas[i])
    axs[1].plot(t, qd[:, i], label=juntas[i])
    axs[2].plot(t, qdd[:, i], label=juntas[i])

axs[0].set_ylabel("Posição [rad/m]")
axs[1].set_ylabel("Velocidade [rad/s or m/s]")
axs[2].set_ylabel("Aceleração [rad/s² or m/s²]")
for ax in axs:
    ax.set_xlabel("Tempo [s]")
    ax.grid(True)
    ax.legend()

plt.tight_layout()
plt.show()

# Graficos de torque
plt.figure(figsize=(8,4))
for i in range(3):
    plt.plot(t, tau[:, i], label=juntas[i])
plt.xlabel("Tempo [s]")
plt.ylabel("Torque [Nm]")
plt.title("Torque em cada junta")
plt.grid(True)
plt.legend()
plt.show()

# ================== Plotando animação ==================
robot.plot(traj, backend="pyplot", dt=dt)
