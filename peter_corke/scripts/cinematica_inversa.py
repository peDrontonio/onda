import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
from spatialmath import SE3

# --------- parâmetros do robô  ----------
l1, l2, l3 = 0.10, 0.10, 0.10
l4, l5 = 0.05, 0.10
theta4_star = np.deg2rad(0.0)   # offset fixo da junta 4 (se houver)
d4_limits = (0.00, 0.30)        # metros
# -----------------------------------------------

def build_robot():
    links = [
        RevoluteDH(a=0.0,  alpha=np.pi/2, d=l1),
        RevoluteDH(a=l2,   alpha=0.0,     d=0.0),
        RevoluteDH(a=l3,   alpha=0.0,     d=0.0),
        PrismaticDH(a=0.0, alpha=-np.pi/2, theta=theta4_star, qlim=d4_limits, offset=l4),  # d = l4 + q
        RevoluteDH(a=0.0,  alpha=np.pi/2, d=l5),
    ]
    return DHRobot(links, name="RRRPR")

def ik_rrrpr_analytic(Td, elbow="up"):
    R = Td.R; x,y,z = Td.t
    # 1) punho
    R01, R11, R21 = R[0,1], R[1,1], R[2,1]
    R20, R22 = R[2,0], R[2,2]
    s234_mag = np.hypot(R01, R11)

    if s234_mag < 1e-9:
        theta1 = np.arctan2(y, x)
        theta5 = np.arctan2(R[1,0], R[0,0]) - theta1
        c234, s234 = np.sign(R21), 0.0
        theta234 = np.arctan2(s234, c234)
    else:
        theta1 = np.arctan2(R11, R01)
        c234 = R21
        s234 = s234_mag
        theta234 = np.arctan2(s234, c234)
        theta5 = np.arctan2(R22, R20)

    c1, s1 = np.cos(theta1), np.sin(theta1)

    # 2) projeção e 2R
    A  = c1*x + s1*y
    Ap = A + l5*s234
    zp = z - l1 - l5*c234

    r2 = Ap**2 + zp**2
    c3 = (r2 - l2**2 - l3**2) / (2*l2*l3)
    c3 = np.clip(c3, -1.0, 1.0)
    s3 = np.sqrt(max(0.0, 1 - c3**2))
    if elbow == "down":
        s3 = -s3
    theta3 = np.arctan2(s3, c3)
    theta2 = np.arctan2(zp, Ap) - np.arctan2(l3*s3, l2 + l3*c3)

    # 3) junta prismática
    D4 = s1*x - c1*y                # = l4 + d4
    d4 = D4 - l4
    # checar limitesq_true
    if not (d4_limits[0]-1e-6 <= d4 <= d4_limits[1]+1e-6):
        raise ValueError(f"d4 fora dos limites: {d4:.3f} m")

    # consistência de orientação
    theta_sum = (theta2 + theta3)
    if abs(((theta_sum - (theta234 - theta4_star) + np.pi) % (2*np.pi)) - np.pi) > 1e-2:
        # pequena correção de 2π se necessário
        pass

    q = np.array([theta1, theta2, theta3, d4, theta5])
    return q

# ------------------- demo -------------------
if __name__ == "__main__":
    robot = build_robot()
    robot.qz = np.zeros(5)  

    # alvo gerado de uma configuração "verdadeira"
    q_true = np.array([np.deg2rad(40), np.deg2rad(20), np.deg2rad(-35), 0.06, np.deg2rad(-20)])
    T_goal = robot.fkine(q_true)

    q_hat = ik_rrrpr_analytic(T_goal, elbow="up")
    print("q_true(deg):", np.rad2deg(q_true[:3]).round(2), q_true[3].round(3), np.rad2deg(q_true[4]).round(2))
    print("q_hat (deg):", np.rad2deg(q_hat[:3]).round(2), q_hat[3].round(3), np.rad2deg(q_hat[4]).round(2))

    # visualização da trajetória
    N = 200
    traj = rtb.jtraj(robot.qz, q_hat, N)
    robot.plot(traj.q)
