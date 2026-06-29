import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.widgets import TextBox, Button
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH, jtraj
from spatialmath import SE3

# =======================================
# 1) ROBOT DEFINITION
# =======================================

l1, l2, l3, l4, l5 = 0.4, 0.8, 0.7, 0.2, 0.3
d4_lim = (0.0, 0.30)
q1_lim = (-np.pi, np.pi)
q2_lim = (-np.pi / 2, np.pi / 2)
q3_lim = (-2 * np.pi / 3, 2 * np.pi / 3)
q5_lim = (-np.pi, np.pi)

arm = DHRobot([
    RevoluteDH(a=0.0, d=l1, alpha=np.pi / 2,  qlim=q1_lim),
    RevoluteDH(a=l2,  d=0.0, alpha=0.0,        qlim=q2_lim),
    RevoluteDH(a=l3,  d=0.0, alpha=0.0,        qlim=q3_lim),
    PrismaticDH(a=0.0, theta=0.0, alpha=-np.pi / 2, offset=l4, qlim=d4_lim),
    RevoluteDH(a=0.0, d=l5, alpha=np.pi / 2,   qlim=q5_lim),
], name="RRRPR")

q_current = np.zeros(5)


def forward_chain_points(q):
    points = [np.array([0.0, 0.0, 0.0])]
    T = np.eye(4)
    for i in range(arm.n):
        T = T @ arm.links[i].A(q[i]).A
        points.append(T[0:3, 3])
    pts = np.array(points)
    return pts[:, 0], pts[:, 1], pts[:, 2]


# =======================================
# 2) FIGURE LAYOUT
# =======================================

fig = plt.figure(figsize=(10, 7))
fig.suptitle("RRRPR – Posição cartesiana desejada", fontsize=12)

ax3d = fig.add_axes([0.03, 0.28, 0.60, 0.66], projection='3d')
ax3d.set_xlabel("X (m)")
ax3d.set_ylabel("Y (m)")
ax3d.set_zlabel("Z (m)")
ax3d.set_xlim(-2, 2)
ax3d.set_ylim(-2, 2)
ax3d.set_zlim(-1.5, 1.5)
ax3d.set_title("Visualização 3D", fontsize=10)

ax_info = fig.add_axes([0.67, 0.28, 0.30, 0.66])
ax_info.axis("off")

# Input row: labels + text boxes + button
ax_lbl_x  = fig.add_axes([0.03, 0.17, 0.04, 0.05])
ax_lbl_x.axis("off")
ax_lbl_x.text(0.5, 0.5, "X (m):", ha='center', va='center', fontsize=10)

ax_box_x  = fig.add_axes([0.08, 0.17, 0.12, 0.05])

ax_lbl_y  = fig.add_axes([0.22, 0.17, 0.04, 0.05])
ax_lbl_y.axis("off")
ax_lbl_y.text(0.5, 0.5, "Y (m):", ha='center', va='center', fontsize=10)

ax_box_y  = fig.add_axes([0.27, 0.17, 0.12, 0.05])

ax_lbl_z  = fig.add_axes([0.41, 0.17, 0.04, 0.05])
ax_lbl_z.axis("off")
ax_lbl_z.text(0.5, 0.5, "Z (m):", ha='center', va='center', fontsize=10)

ax_box_z  = fig.add_axes([0.46, 0.17, 0.12, 0.05])

ax_btn    = fig.add_axes([0.20, 0.07, 0.22, 0.07])

# =======================================
# 3) DRAW INITIAL ARM
# =======================================

x0, y0, z0 = forward_chain_points(q_current)
line_robot, = ax3d.plot(x0, y0, z0, "o-", linewidth=3, color="tab:red")
target_dot, = ax3d.plot([], [], [], "g*", markersize=12)

info_text = ax_info.text(
    0.0, 1.0, "",
    va="top", ha="left", fontsize=9,
    transform=ax_info.transAxes,
    family="monospace",
)


def _info_str(q, status=""):
    T = arm.fkine(q)
    p = T.t
    info = (
        "Configuração atual q:\n"
        f"  q1 = {q[0]: .3f} rad\n"
        f"  q2 = {q[1]: .3f} rad\n"
        f"  q3 = {q[2]: .3f} rad\n"
        f"  d4 = {q[3]: .4f} m\n"
        f"  q5 = {q[4]: .3f} rad\n\n"
        "Pose do efetuador:\n"
        f"  x = {p[0]: .4f} m\n"
        f"  y = {p[1]: .4f} m\n"
        f"  z = {p[2]: .4f} m\n"
    )
    if status:
        info += f"\n{status}"
    return info


def update_display(q, status=""):
    x, y, z = forward_chain_points(q)
    line_robot.set_data(x, y)
    line_robot.set_3d_properties(z)
    info_text.set_text(_info_str(q, status))
    fig.canvas.draw_idle()


# =======================================
# 4) INPUT WIDGETS
# =======================================

T0 = arm.fkine(q_current)
p0 = T0.t

tb_x = TextBox(ax_box_x, '', initial=f"{p0[0]:.3f}")
tb_y = TextBox(ax_box_y, '', initial=f"{p0[1]:.3f}")
tb_z = TextBox(ax_box_z, '', initial=f"{p0[2]:.3f}")

btn_go = Button(ax_btn, "Ir para posição")


# =======================================
# 5) GO-TO-POSITION CALLBACK
# =======================================

def go_to_position(event):
    global q_current

    try:
        x = float(tb_x.text)
        y = float(tb_y.text)
        z = float(tb_z.text)
    except ValueError:
        update_display(q_current, status="ERRO: entrada inválida.")
        return

    # show target marker
    target_dot.set_data([x], [y])
    target_dot.set_3d_properties([z])

    sol = arm.ikine_LM(SE3(x, y, z), mask=[1, 1, 1, 0, 0, 0], q0=q_current)

    if not sol.success:
        target_dot.set_data([], [])
        target_dot.set_3d_properties([])
        update_display(q_current, status="ERRO: posição inalcançável.")
        return

    q_target = sol.q
    traj = jtraj(q_current, q_target, 80)

    for q in traj.q:
        q_current = q
        x_c, y_c, z_c = forward_chain_points(q_current)
        line_robot.set_data(x_c, y_c)
        line_robot.set_3d_properties(z_c)
        info_text.set_text(_info_str(q_current))
        fig.canvas.draw()
        plt.pause(0.02)

    # update text boxes to reflect achieved position
    p_final = arm.fkine(q_current).t
    tb_x.set_val(f"{p_final[0]:.3f}")
    tb_y.set_val(f"{p_final[1]:.3f}")
    tb_z.set_val(f"{p_final[2]:.3f}")

    update_display(q_current, status="OK: posição alcançada.")


btn_go.on_clicked(go_to_position)

# =======================================
# 6) STARTUP & SHOW
# =======================================

update_display(q_current)
plt.show()
