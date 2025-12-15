import numpy as np
import tkinter as tk
from tkinter import ttk
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH, jtraj

# ============================================
# 1) DEFINIÇÃO DO MANIPULADOR RRRPR
# ============================================

# Comprimentos (m) — mesmos que combinamos
l1, l2, l3, l4, l5 = 0.4, 0.8, 0.7, 0.2, 0.3

# Limites das juntas
q1_lim = (-np.pi, np.pi)           # base: -180° a 180°
q2_lim = (-np.pi/2, np.pi/2)       # ombro: -90° a 90°
q3_lim = (-2*np.pi/3, 2*np.pi/3)   # cotovelo: -120° a 120°
d4_lim = (0.0, 0.30)               # curso prismática: 0 a 30 cm
q5_lim = (-np.pi, np.pi)           # rotação tool: -180° a 180°

# Monta o robô com DH padrão
arm = DHRobot([
    RevoluteDH(a=0.0, d=l1, alpha=np.pi/2, qlim=q1_lim),
    RevoluteDH(a=l2,  d=0.0, alpha=0.0,     qlim=q2_lim),
    RevoluteDH(a=l3,  d=0.0, alpha=0.0,     qlim=q3_lim),
    PrismaticDH(a=0.0, theta=0.0, alpha=-np.pi/2, offset=l4, qlim=d4_lim),
    RevoluteDH(a=0.0, d=l5, alpha=np.pi/2, qlim=q5_lim),
], name="Subsea_RRRPR")

print(arm)
print("qlim:\n", arm.qlim)

# Estado atual das juntas (q1, q2, q3, d4, q5)
q_current = np.array([0.0, 0.0, 0.0, 0.0, 0.0])


# ============================================
# 2) INTERFACE TKINTER COM SLIDERS
# ============================================

root = tk.Tk()
root.title("Controle do braço subaquático RRRPR (sem Swift)")

mainframe = ttk.Frame(root, padding="10 10 10 10")
mainframe.grid(row=0, column=0, sticky="nsew")

root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

# Variáveis ligadas aos sliders
q1_var = tk.DoubleVar(value=q_current[0])
q2_var = tk.DoubleVar(value=q_current[1])
q3_var = tk.DoubleVar(value=q_current[2])
d4_var = tk.DoubleVar(value=q_current[3])
q5_var = tk.DoubleVar(value=q_current[4])

# Texto pro Jacobiano e pose
jacobian_text = tk.StringVar()
pose_text = tk.StringVar()
jacobian_text.set("Jacobiano atual:\n")
pose_text.set("Pose atual do efetuador:\n")


def get_q_from_vars():
    """Lê os valores das juntas a partir dos sliders."""
    return np.array([
        q1_var.get(),
        q2_var.get(),
        q3_var.get(),
        d4_var.get(),
        q5_var.get()
    ])


def update_robot_state(*args):
    """
    Atualiza o estado do robô (cálculo do Jacobiano e fkine)
    com base nos sliders. Não tem visualização, só cálculo.
    """
    global q_current
    q_current = get_q_from_vars()

    # Jacobiano na base
    J = arm.jacob0(q_current)

    # Pose do efetuador
    T = arm.fkine(q_current)
    p = T.t              # posição (x, y, z)
    # Usa angulos de Euler RPY (ZYX por default)
    rpy = T.rpy(order="zyx", unit="deg")

    jacobian_text.set(f"Jacobiano atual (6x5):\n{np.round(J, 3)}")
    pose_text.set(
        "Pose atual do efetuador:\n"
        f"  Posição (m): x={p[0]: .3f}, y={p[1]: .3f}, z={p[2]: .3f}\n"
        f"  Orientação RPY (graus, ZYX):\n"
        f"    roll  (γ) = {rpy[2]: .2f}\n"
        f"    pitch (β) = {rpy[1]: .2f}\n"
        f"    yaw   (α) = {rpy[0]: .2f}"
    )


# Sliders das juntas
ttk.Label(mainframe, text="q1 (rad)").grid(column=0, row=0, sticky="w")
s_q1 = ttk.Scale(mainframe, from_=q1_lim[0], to=q1_lim[1],
                 orient='horizontal', variable=q1_var, command=update_robot_state)
s_q1.grid(column=1, row=0, sticky="ew")

ttk.Label(mainframe, text="q2 (rad)").grid(column=0, row=1, sticky="w")
s_q2 = ttk.Scale(mainframe, from_=q2_lim[0], to=q2_lim[1],
                 orient='horizontal', variable=q2_var, command=update_robot_state)
s_q2.grid(column=1, row=1, sticky="ew")

ttk.Label(mainframe, text="q3 (rad)").grid(column=0, row=2, sticky="w")
s_q3 = ttk.Scale(mainframe, from_=q3_lim[0], to=q3_lim[1],
                 orient='horizontal', variable=q3_var, command=update_robot_state)
s_q3.grid(column=1, row=2, sticky="ew")

ttk.Label(mainframe, text="d4 (m)").grid(column=0, row=3, sticky="w")
s_q4 = ttk.Scale(mainframe, from_=d4_lim[0], to=d4_lim[1],
                 orient='horizontal', variable=d4_var, command=update_robot_state)
s_q4.grid(column=1, row=3, sticky="ew")

ttk.Label(mainframe, text="q5 (rad)").grid(column=0, row=4, sticky="w")
s_q5 = ttk.Scale(mainframe, from_=q5_lim[0], to=q5_lim[1],
                 orient='horizontal', variable=q5_var, command=update_robot_state)
s_q5.grid(column=1, row=4, sticky="ew")

# Labels de Jacobiano e Pose
jacobian_label = ttk.Label(mainframe, textvariable=jacobian_text, justify="left")
jacobian_label.grid(column=0, row=5, columnspan=2, sticky="w", pady=(10, 5))

pose_label = ttk.Label(mainframe, textvariable=pose_text, justify="left")
pose_label.grid(column=0, row=6, columnspan=2, sticky="w", pady=(0, 10))


# ============================================
# 3) ANIMAR TRAJETÓRIA (APENAS NUMÉRICA)
# ============================================

def animate_trajectory():
    """
    Gera uma trajetória de q_current até um alvo, e vai
    atualizando os sliders + Jacobiano + pose ao longo do tempo.
    """
    global q_current

    # alvo (você pode tune-ar isso depois pra algo "físico")
    q_target = np.array([
        0.4,              # q1: gira base
        -0.4,             # q2: baixa ombro
        0.25,             # q3: dobra cotovelo
        0.18,             # d4: estende tool
        np.pi/2           # q5: rotaciona tool
    ])

    steps = 80
    traj = jtraj(q_current, q_target, steps)

    def step_traj(i=0):
        global q_current
        if i >= steps:
            return

        q_current = traj.q[i, :]

        # atualiza sliders (isso dispara update_robot_state pelo 'command')
        q1_var.set(float(q_current[0]))
        q2_var.set(float(q_current[1]))
        q3_var.set(float(q_current[2]))
        d4_var.set(float(q_current[3]))
        q5_var.set(float(q_current[4]))

        # agenda próximo frame
        root.after(30, lambda: step_traj(i + 1))

    step_traj(0)


anim_button = ttk.Button(mainframe, text="Animar trajetória", command=animate_trajectory)
anim_button.grid(column=0, row=7, columnspan=2, pady=(5, 0))

# Configura expansão da UI
for col in range(2):
    mainframe.columnconfigure(col, weight=1)

# Calcula Jacobiano/pose iniciais
update_robot_state()

root.mainloop()