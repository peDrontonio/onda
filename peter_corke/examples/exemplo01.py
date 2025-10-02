import roboticstoolbox as rtb
import numpy as np

# Cria o Puma 560
puma = rtb.models.DH.Puma560()
print(puma)
# Settando uma posição de junta
q = np.array([ 0, np.pi/2, -np.pi/2, 0, 0, 0])

# plotando o manipulador
puma.plot(q, block=True, backend="pyplot")