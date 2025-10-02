import roboticstoolbox as rtb
import numpy as np

# Criando um DH robot
# Na questão 2 da lista de cinematica direta, temos o manipulador
#
#                        DH table
#           +--------+---------+------+-------+
#           | theta  |    d    |  a   | alpha |
#           +--------+---------+------+-------+
#  Junta 1  | theta1 | l0 + l2 |  l1  |   0   |
#  Junta 2  | theta2 |    0    |  l3  |   0   |
#  Junta 3  |   0    | -l4 -d3 |   0  |   pi  |
#           +--------+---------+------+-------+

# Dados do manipulador
l0 = 1
l1 = 1
l2 = 1
l3 = 1
l4 = 1

# Declarando o robo pelos parametros de DH
robot = rtb.DHRobot([
    rtb.RevoluteDH(d=(l0 + l2), a=l1),
    rtb.RevoluteDH(a=l3),
    rtb.PrismaticDH(offset=-l4, alpha=-np.pi)
])
print(robot)
# Definindo uma posicao de junta
q = np.array([0, 0, 0])

# Exibindo o robo
robot.plot(q, backend="pyplot", block=True)