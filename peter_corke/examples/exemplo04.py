import roboticstoolbox as rtb
import numpy as np

# Mesma definição do robô
l0 = 1
l1 = 1
l2 = 1
l3 = 1
l4 = 1

robot = rtb.DHRobot([
    rtb.RevoluteDH(d=(l0 + l2), a=l1),
    rtb.RevoluteDH(a=l3),
    rtb.PrismaticDH(offset=-l4, alpha=-np.pi)
])

# Posicoes inicial e final
q_start = np.array([0, 0, 0])
q_end = np.array([np.pi/4, np.pi/3, -0.2])

# Tempo de simulacao e dt
time_s = 5.0
dt = 0.05
steps = int(time_s / dt)

# Vetor de tempo
t = np.linspace(0, time_s, steps)

# Trajetória polinomial de 5º grau
traj = rtb.jtraj(q_start, q_end, t)

a=robot.fkine(traj.q[1])
print(a)
# Plotando animação
robot.plot(traj.q, backend="pyplot", dt=dt)
