import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH, jtraj
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.widgets import Slider, Button

# =======================================
# 1) MANIPULADOR RRRPR (braço subaquático)
# =======================================

l1, l2, l3, l4, l5 = 0.4, 0.8, 0.7, 0.2, 0.3
d4_lim = (0.0, 0.30)

q1_lim = (-np.pi, np.pi)
q2_lim = (-np.pi/2, np.pi/2)
q3_lim = (-2*np.pi/3, 2*np.pi/3)
q5_lim = (-np.pi, np.pi)

arm = DHRobot([
    RevoluteDH(a=0.0, d=l1, alpha=np.pi/2, qlim=q1_lim),
    RevoluteDH(a=l2,  d=0.0, alpha=0.0,     qlim=q2_lim),
    RevoluteDH(a=l3,  d=0.0, alpha=0.0,     qlim=q3_lim),
    PrismaticDH(a=0.0, theta=0.0, alpha=-np.pi/2, offset=l4, qlim=d4_lim),
    RevoluteDH(a=0.0, d=l5, alpha=np.pi/2, qlim=q5_lim),
], name="RRRPR")

print(arm)


# =======================================
# 2) CINEMÁTICA PARA DESENHAR O BRAÇO
# =======================================

def forward_chain_points(q):
    """
    Retorna arrays x,y,z com os pontos da cadeia cinemática:
    base -> junta1 -> ... -> efetuador
    """
    points = [np.array([0.0, 0.0, 0.0])]
    T = np.eye(4)

    for i in range(arm.n):
        T = T @ arm.links[i].A(q[i]).A
        p = T[0:3, 3]
        points.append(p)

    pts = np.array(points)  # shape (6,3)
    return pts[:, 0], pts[:, 1], pts[:, 2]


def pose_and_jacobian(q):
    """Retorna (T, J) pra essa configuração."""
    T = arm.fkine(q)
    J = arm.jacob0(q)
    return T, J


# =======================================
# 3) FIGURA 3D + SLIDERS
# =======================================

# Layout da figura
fig = plt.figure(figsize=(8, 6))
ax3d = fig.add_axes([0.05, 0.3, 0.6, 0.65], projection='3d')
ax_info = fig.add_axes([0.7, 0.3, 0.28, 0.65])
ax_info.axis("off")

ax3d.set_title("Manipulador RRRPR – Visualização 3D")
ax3d.set_xlabel("X")
ax3d.set_ylabel("Y")
ax3d.set_zlabel("Z")
ax3d.set_xlim(-2, 2)
ax3d.set_ylim(-2, 2)
ax3d.set_zlim(-1.5, 1.5)

# sliders na parte inferior
ax_q1 = fig.add_axes([0.10, 0.22, 0.6, 0.03])
ax_q2 = fig.add_axes([0.10, 0.18, 0.6, 0.03])
ax_q3 = fig.add_axes([0.10, 0.14, 0.6, 0.03])
ax_q4 = fig.add_axes([0.10, 0.10, 0.6, 0.03])
ax_q5 = fig.add_axes([0.10, 0.06, 0.6, 0.03])

# botão de animação
ax_button = fig.add_axes([0.75, 0.10, 0.18, 0.08])

# Estado inicial
q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

# Cria sliders
s_q1 = Slider(ax_q1, "q1 (rad)", q1_lim[0], q1_lim[1], valinit=q0[0])
s_q2 = Slider(ax_q2, "q2 (rad)", q2_lim[0], q2_lim[1], valinit=q0[1])
s_q3 = Slider(ax_q3, "q3 (rad)", q3_lim[0], q3_lim[1], valinit=q0[2])
s_q4 = Slider(ax_q4, "d4 (m)",   d4_lim[0], d4_lim[1], valinit=q0[3])
s_q5 = Slider(ax_q5, "q5 (rad)", q5_lim[0], q5_lim[1], valinit=q0[4])

btn_anim = Button(ax_button, "Animar\ntrajetória")


# =======================================
# 4) OBJETOS GRÁFICOS: BRAÇO + TRAJETÓRIA
# =======================================

x0, y0, z0 = forward_chain_points(q0)
line_robot, = ax3d.plot(x0, y0, z0, "o-", linewidth=3, color="tab:red")
traj_line, = ax3d.plot([], [], [], "-", linewidth=2, color="tab:gray")

info_text = ax_info.text(
    0.0, 1.0, "",
    va="top", ha="left", fontsize=9,
    transform=ax_info.transAxes,
)

trajectory_points = []  # guarda trajetória da ponta


def get_q_from_sliders():
    return np.array([
        s_q1.val,
        s_q2.val,
        s_q3.val,
        s_q4.val,
        s_q5.val,
    ])


def update_from_sliders(val=None):
    """Atualiza o braço 3D, Jacobiano e pose com base nos sliders."""
    global trajectory_points
    q = get_q_from_sliders()

    # atualiza braço 3D
    x, y, z = forward_chain_points(q)
    line_robot.set_data(x, y)
    line_robot.set_3d_properties(z)

    # atualiza info
    T, J = pose_and_jacobian(q)
    p = T.t
    rpy = T.rpy(order="zyx", unit="deg")

    info_str = (
        "Configuração atual q:\n"
        f"  q1 = {q[0]: .3f} rad\n"
        f"  q2 = {q[1]: .3f} rad\n"
        f"  q3 = {q[2]: .3f} rad\n"
        f"  d4 = {q[3]: .3f} m\n"
        f"  q5 = {q[4]: .3f} rad\n\n"
        "Pose do efetuador:\n"
        f"  x = {p[0]: .3f} m\n"
        f"  y = {p[1]: .3f} m\n"
        f"  z = {p[2]: .3f} m\n"
        f"  yaw (α)   = {rpy[0]: .2f} °\n"
        f"  pitch (β) = {rpy[1]: .2f} °\n"
        f"  roll (γ)  = {rpy[2]: .2f} °\n\n"
        "Jacobiano J (6x5):\n"
        f"{np.round(J, 3)}"
    )

    info_text.set_text(info_str)

    # opcional: atualizar trajetória com ponto atual quando mexer
    # trajectory_points.append([x[-1], y[-1], z[-1]])
    # p_traj = np.array(trajectory_points)
    # traj_line.set_data(p_traj[:, 0], p_traj[:, 1])
    # traj_line.set_3d_properties(p_traj[:, 2])

    fig.canvas.draw_idle()


# conecta sliders
s_q1.on_changed(update_from_sliders)
s_q2.on_changed(update_from_sliders)
s_q3.on_changed(update_from_sliders)
s_q4.on_changed(update_from_sliders)
s_q5.on_changed(update_from_sliders)


# =======================================
# 5) ANIMAR TRAJETÓRIA
# =======================================

def on_click_anim(event):
    """Gera uma trajetória e anima o braço + rastro da ponta."""
    global trajectory_points

    q_start = get_q_from_sliders()
    q_target = np.array([
        0.5,
        -0.6,
        0.6,
        0.2,
        1.2
    ])

    steps = 120
    traj = jtraj(q_start, q_target, steps)

    trajectory_points = []

    for q in traj.q:
        # seta sliders (isso já chama update_from_sliders)
        s_q1.set_val(float(q[0]))
        s_q2.set_val(float(q[1]))
        s_q3.set_val(float(q[2]))
        s_q4.set_val(float(q[3]))
        s_q5.set_val(float(q[4]))

        # rastro da ponta do efetuador
        x, y, z = forward_chain_points(q)
        trajectory_points.append([x[-1], y[-1], z[-1]])
        p_traj = np.array(trajectory_points)
        traj_line.set_data(p_traj[:, 0], p_traj[:, 1])
        traj_line.set_3d_properties(p_traj[:, 2])

        plt.pause(0.02)  # controla velocidade da animação


btn_anim.on_clicked(on_click_anim)


# =======================================
# 6) MOSTRA JANELA
# =======================================

update_from_sliders()  # inicializa info
plt.show()
