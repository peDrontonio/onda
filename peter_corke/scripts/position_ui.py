"""
RRRPR Braco — interactive position UI.

Type X, Y, Z in the text boxes and click "Ir para posição".
The 3-D view animates the arm moving along the computed trajectory.
Joint values and end-effector pose are shown in the side panel.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Button

from cinematica_inversa import fk, fk_position, chain_points, ik, JOINT_NAMES


# ── State ─────────────────────────────────────────────────────────────────────
q_current = np.zeros(5)


# ── Figure layout ─────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(10, 7))
fig.suptitle("Braço RRRPR – Posição cartesiana desejada", fontsize=12)

ax3d = fig.add_axes([0.03, 0.28, 0.60, 0.66], projection='3d')
ax3d.set_xlabel("X (m)"); ax3d.set_ylabel("Y (m)"); ax3d.set_zlabel("Z (m)")
ax3d.set_xlim(-0.5, 0.5); ax3d.set_ylim(-0.5, 0.5); ax3d.set_zlim(-0.1, 0.6)
ax3d.set_title("Visualização 3D", fontsize=10)

ax_info = fig.add_axes([0.67, 0.28, 0.30, 0.66])
ax_info.axis("off")

# Input row
for col, label, ax_lbl_pos in zip(
    ['X', 'Y', 'Z'],
    [0.03, 0.22, 0.41],
    [0.03, 0.22, 0.41],
):
    a = fig.add_axes([ax_lbl_pos, 0.17, 0.04, 0.05])
    a.axis("off")
    a.text(0.5, 0.5, f"{col} (m):", ha='center', va='center', fontsize=10)

ax_box_x = fig.add_axes([0.08, 0.17, 0.12, 0.05])
ax_box_y = fig.add_axes([0.27, 0.17, 0.12, 0.05])
ax_box_z = fig.add_axes([0.46, 0.17, 0.12, 0.05])
ax_btn   = fig.add_axes([0.20, 0.07, 0.22, 0.07])


# ── Initial arm drawing ───────────────────────────────────────────────────────
x0, y0, z0 = chain_points(q_current)
line_robot, = ax3d.plot(x0, y0, z0, "o-", linewidth=3, color="tab:red")
target_dot, = ax3d.plot([], [], [], "g*", markersize=12)

info_text = ax_info.text(
    0.0, 1.0, "",
    va="top", ha="left", fontsize=9,
    transform=ax_info.transAxes,
    family="monospace",
)


def _info_str(q, status=""):
    p = fk_position(q)
    T = fk(q)
    lines = [
        "Configuração atual q:",
        f"  q1 = {q[0]: .3f} rad",
        f"  q2 = {q[1]: .3f} rad",
        f"  q3 = {q[2]: .3f} rad",
        f"  d4 = {q[3]*1000: .1f} mm",
        f"  q5 = {q[4]: .3f} rad",
        "",
        "Pose do efetuador:",
        f"  x = {p[0]: .4f} m",
        f"  y = {p[1]: .4f} m",
        f"  z = {p[2]: .4f} m",
    ]
    if status:
        lines += ["", status]
    return "\n".join(lines)


def update_display(q, status=""):
    xc, yc, zc = chain_points(q)
    line_robot.set_data(xc, yc)
    line_robot.set_3d_properties(zc)
    info_text.set_text(_info_str(q, status))
    fig.canvas.draw_idle()


# ── Input widgets ─────────────────────────────────────────────────────────────
p0 = fk_position(q_current)
tb_x = TextBox(ax_box_x, '', initial=f"{p0[0]:.3f}")
tb_y = TextBox(ax_box_y, '', initial=f"{p0[1]:.3f}")
tb_z = TextBox(ax_box_z, '', initial=f"{p0[2]:.3f}")
btn_go = Button(ax_btn, "Ir para posição")


# ── Go-to-position callback ───────────────────────────────────────────────────
def go_to_position(event):
    global q_current

    try:
        x, y, z = float(tb_x.text), float(tb_y.text), float(tb_z.text)
    except ValueError:
        update_display(q_current, status="ERRO: entrada inválida.")
        return

    target_dot.set_data([x], [y])
    target_dot.set_3d_properties([z])

    q_target, ok, err = ik((x, y, z), q_init=q_current.copy())

    if not ok:
        target_dot.set_data([], [])
        target_dot.set_3d_properties([])
        update_display(q_current, status=f"ERRO: posição inalcançável.\n(erro={err*1000:.1f} mm)")
        return

    # Animate trajectory
    n_frames = 60
    for alpha in np.linspace(0, 1, n_frames):
        q_interp = q_current + alpha * (q_target - q_current)
        xc, yc, zc = chain_points(q_interp)
        line_robot.set_data(xc, yc)
        line_robot.set_3d_properties(zc)
        info_text.set_text(_info_str(q_interp))
        fig.canvas.draw()
        plt.pause(0.015)

    q_current = q_target.copy()
    p_final = fk_position(q_current)
    tb_x.set_val(f"{p_final[0]:.3f}")
    tb_y.set_val(f"{p_final[1]:.3f}")
    tb_z.set_val(f"{p_final[2]:.3f}")
    update_display(q_current, status=f"OK: posição alcançada.\n(erro IK = {err*1000:.3f} mm)")


btn_go.on_clicked(go_to_position)

# ── Start ─────────────────────────────────────────────────────────────────────
update_display(q_current)
plt.show()
