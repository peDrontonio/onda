import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import swift
import time

# 1) DEFINIÇÃO DO BRAÇO
l1, l2, l3, l4, l5 = 0.4, 0.8, 0.7, 0.2, 0.3
q1_lim = (-np.pi, np.pi)
q2_lim = (-np.pi/2, np.pi/2)
q3_lim = (-2*np.pi/3, 2*np.pi/3)
d4_lim = (0.0, 0.30)
q5_lim = (-np.pi, np.pi)

arm = DHRobot([
    RevoluteDH(a=0.0, d=l1, alpha=np.pi/2, qlim=q1_lim),
    RevoluteDH(a=l2, d=0.0, alpha=0.0,     qlim=q2_lim),
    RevoluteDH(a=l3, d=0.0, alpha=0.0,     qlim=q3_lim),
    PrismaticDH(a=0.0, theta=0.0, alpha=-np.pi/2, offset=l4, qlim=d4_lim),
    RevoluteDH(a=0.0, d=l5, alpha=np.pi/2, qlim=q5_lim),
], name="Subsea_RRRPR")

print("Robot defined:", arm)

# Check geometry
print("Checking geometry...")
for i, link in enumerate(arm.links):
    print(f"Link {i}: {link}")
    print(f"  Geometry: {link.geometry}")
    print(f"  Collision: {link.collision}")

# 2) SWIFT
print("Launching Swift...")
env = swift.Swift()
env.launch(realtime=True)

print("Adding robot to env...")
env.add(arm)

print("Stepping environment...")
for i in range(100):
    env.step(0.05)
    time.sleep(0.05)

print("Done. Check browser.")
