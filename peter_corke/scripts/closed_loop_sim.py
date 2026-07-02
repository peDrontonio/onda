#!/usr/bin/env python3
"""
Closed-loop forward-dynamics simulation of the Braco RRRPR manipulator, built on
Peter Corke's **Robotics Toolbox** (the model is imported from the project URDF,
see `braco_rtb.py`).

Control law: τ = Kp·(q_des − q) + Kd·(q̇_des − q̇) + G(q)   [PD + gravity comp.]

The plant integrates the full nonlinear rigid-body dynamics (all terms from the
URDF via RTB's recursive Newton–Euler):

    q̈ = M(q)⁻¹ · [τ − C(q,q̇)·q̇ − G(q)]

Reference is a quintic polynomial trajectory from home (q=0) to a target pose
computed by RTB inverse kinematics.  Integration uses SciPy `solve_ivp` with the
stiff LSODA method (the high-gain closed loop is stiff, so explicit steppers are
either unstable or extremely slow with the full RTB dynamics).

Run (inside the project venv):
    python closed_loop_sim.py [x] [y] [z] [--duration T] [--output DIR]

Saves plots to:  output/
"""

import argparse
import os
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from braco_rtb import (
    ik, fk_position, JOINT_NAMES,
    gravload, plant_accel,
)


# ── PD + gravity compensation gains ───────────────────────────────────────────

KP = np.diag([120.0, 100.0, 80.0, 500.0, 30.0])
KD = np.diag([ 10.0,   8.0,  6.0,  30.0,  2.0])


def control_law(q, qd, q_des, qd_des):
    """τ = Kp·e + Kd·ė + G(q)  — G(q) from the Robotics Toolbox model."""
    e  = q_des - q
    ed = qd_des - qd
    return KP @ e + KD @ ed + gravload(q)


# ── Quintic polynomial reference ───────────────────────────────────────────────

def quintic_coeffs(q0, qf, T):
    """Return 6 coefficients a0..a5 for each joint (shape 5×6)."""
    dq = qf - q0
    T3, T4, T5 = T**3, T**4, T**5
    a = np.zeros((5, 6))
    a[:, 0] = q0
    a[:, 3] = 10 * dq / T3
    a[:, 4] = -15 * dq / T4
    a[:, 5] =  6 * dq / T5
    return a


def quintic_eval(a, t):
    """Evaluate position, velocity, acceleration at time t."""
    t2, t3, t4, t5 = t**2, t**3, t**4, t**5
    q   = a[:,0] + a[:,1]*t + a[:,2]*t2 + a[:,3]*t3 + a[:,4]*t4 + a[:,5]*t5
    qd  = a[:,1] + 2*a[:,2]*t + 3*a[:,3]*t2 + 4*a[:,4]*t3 + 5*a[:,5]*t4
    qdd = 2*a[:,2] + 6*a[:,3]*t + 12*a[:,4]*t2 + 20*a[:,5]*t3
    return q, qd, qdd


# ── ODE (plant = full RTB dynamics) ────────────────────────────────────────────

def ode(t, state, coeffs, T):
    q  = state[:5]
    qd = state[5:]

    t_clamped = min(t, T)
    q_des, qd_des, _ = quintic_eval(coeffs, t_clamped)

    tau = control_law(q, qd, q_des, qd_des)
    qdd = plant_accel(q, qd, tau)          # M⁻¹(τ − C·q̇ − G), all terms from URDF

    return np.concatenate([qd, qdd])


# ── Plots ─────────────────────────────────────────────────────────────────────

JOINT_LABELS = ['θ₁ (base_rot1)', 'θ₂ (rot1_rot2)', 'θ₃ (rot2_rot3)',
                'd₄ (prism, mm)', 'θ₅ (prism1_rot4)']
COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']


def _q_to_display(i, val):
    """Revolute joints -> degrees, prismatic joint (index 3) -> millimetres."""
    return val * 1000 if i == 3 else np.degrees(val)


def _unit(i):
    return 'mm' if i == 3 else '°'


def plot_tracking(t_eval, q_hist, q_des_hist, out):
    fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('Rastreamento de posição — PD + compensação de gravidade', fontsize=13, fontweight='bold')
    for i, ax in enumerate(axes):
        ax.plot(t_eval, [_q_to_display(i, v) for v in q_des_hist[:, i]],
                '--', color=COLORS[i], linewidth=1.5, label='q_des')
        ax.plot(t_eval, [_q_to_display(i, v) for v in q_hist[:, i]],
                '-', color=COLORS[i], linewidth=2.0, label='q')
        ax.set_ylabel(f'{JOINT_LABELS[i]} ({_unit(i)})', fontsize=9)
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend(fontsize=9)
    axes[-1].set_xlabel('Tempo (s)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out, 'tracking.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  ✓ {path}')


def plot_velocity(t_eval, qd_hist, qd_des_hist, out):
    units = ['°/s', '°/s', '°/s', 'mm/s', '°/s']
    fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('Velocidade das juntas — q̇(t)', fontsize=13, fontweight='bold')
    for i, ax in enumerate(axes):
        ax.plot(t_eval, [_q_to_display(i, v) for v in qd_des_hist[:, i]],
                '--', color=COLORS[i], linewidth=1.5, label='q̇_des')
        ax.plot(t_eval, [_q_to_display(i, v) for v in qd_hist[:, i]],
                '-', color=COLORS[i], linewidth=2.0, label='q̇')
        ax.axhline(0, color='gray', linewidth=0.8, linestyle='--')
        ax.set_ylabel(f'{JOINT_LABELS[i]} ({units[i]})', fontsize=9)
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend(fontsize=9)
    axes[-1].set_xlabel('Tempo (s)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out, 'velocity.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  ✓ {path}')


def plot_error(t_eval, q_hist, q_des_hist, out):
    fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('Erro de rastreamento — e(t) = q_des(t) − q(t)', fontsize=13, fontweight='bold')
    for i, ax in enumerate(axes):
        err = [_q_to_display(i, q_des_hist[k, i] - q_hist[k, i]) for k in range(len(t_eval))]
        ax.plot(t_eval, err, color=COLORS[i], linewidth=2)
        ax.axhline(0, color='gray', linewidth=0.8, linestyle='--')
        ax.set_ylabel(f'e ({_unit(i)})', fontsize=9)
        ax.set_title(JOINT_LABELS[i], fontsize=9, loc='left')
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('Tempo (s)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out, 'tracking_error.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  ✓ {path}')


def plot_torques(t_eval, tau_hist, out):
    units = ['N·m', 'N·m', 'N·m', 'N', 'N·m']
    fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('Torques de controle — τ = Kp·e + Kd·ė + G(q)', fontsize=13, fontweight='bold')
    for i, ax in enumerate(axes):
        ax.plot(t_eval, tau_hist[:, i], color=COLORS[i], linewidth=2)
        peak = np.max(np.abs(tau_hist[:, i]))
        ax.axhline( peak, color='red', linewidth=0.8, linestyle=':', alpha=0.6, label=f'|τ|_max = {peak:.1f}')
        ax.axhline(-peak, color='red', linewidth=0.8, linestyle=':', alpha=0.6)
        ax.axhline(0, color='gray', linewidth=0.8, linestyle='--')
        ax.set_ylabel(f'τ ({units[i]})', fontsize=9)
        ax.set_title(JOINT_LABELS[i], fontsize=9, loc='left')
        ax.legend(fontsize=8, loc='upper right')
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('Tempo (s)', fontsize=11)
    plt.tight_layout()
    path = os.path.join(out, 'torques.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  ✓ {path}')


def plot_summary(t_eval, q_hist, q_des_hist, qd_hist, tau_hist, q_start, q_end, out):
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Simulação em malha fechada — Braço RRRPR (Robotics Toolbox)', fontsize=14, fontweight='bold')

    # Positions
    ax = axes[0, 0]
    for i in range(5):
        disp = [_q_to_display(i, v) for v in q_hist[:, i]]
        des  = [_q_to_display(i, v) for v in q_des_hist[:, i]]
        ax.plot(t_eval, des, '--', color=COLORS[i], linewidth=1.2)
        ax.plot(t_eval, disp, '-', color=COLORS[i], linewidth=2, label=JOINT_LABELS[i].split('(')[0].strip())
    ax.set_title('Posições (— real,  -- referência)')
    ax.set_xlabel('Tempo (s)')
    ax.set_ylabel('Posição (° / mm)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Velocities
    ax = axes[0, 1]
    for i in range(5):
        vel = [_q_to_display(i, v) for v in qd_hist[:, i]]
        ax.plot(t_eval, vel, '-', color=COLORS[i], linewidth=2, label=JOINT_LABELS[i].split('(')[0].strip())
    ax.axhline(0, color='gray', linewidth=0.8, linestyle='--')
    ax.set_title('Velocidades q̇(t)')
    ax.set_xlabel('Tempo (s)')
    ax.set_ylabel('Velocidade (°/s / mm/s)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Torques
    ax = axes[1, 0]
    for i in range(5):
        ax.plot(t_eval, tau_hist[:, i], color=COLORS[i], linewidth=2, label=JOINT_LABELS[i].split('(')[0].strip())
    ax.axhline(0, color='gray', linewidth=0.8, linestyle='--')
    ax.set_title('Torques de controle τ(t)')
    ax.set_xlabel('Tempo (s)')
    ax.set_ylabel('Torque (N·m / N)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # End-effector cartesian error
    ax = axes[1, 1]
    ee_real = np.array([fk_position(q_hist[k]) for k in range(len(t_eval))])
    ee_des  = np.array([fk_position(q_des_hist[k]) for k in range(len(t_eval))])
    ax.plot(t_eval, np.linalg.norm(ee_real - ee_des, axis=1) * 1000, 'k-', linewidth=2)
    ax.set_title('Erro de posição cartesiano do end-effector')
    ax.set_xlabel('Tempo (s)')
    ax.set_ylabel('Erro (mm)')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out, 'summary.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  ✓ {path}')


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Closed-loop PD+G simulation of Braco (Robotics Toolbox)')
    parser.add_argument('x', type=float, nargs='?', default=-0.155, help='Target X (m)')
    parser.add_argument('y', type=float, nargs='?', default=0.290,  help='Target Y (m)')
    parser.add_argument('z', type=float, nargs='?', default=0.361,  help='Target Z (m)')
    parser.add_argument('--duration', '-d', type=float, default=4.0, help='Trajectory duration (s)')
    parser.add_argument('--output', '-o', default='output', help='Output directory for plots')
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)
    T = args.duration

    # Inverse kinematics (Robotics Toolbox, position only)
    q_end, ok, err = ik((args.x, args.y, args.z))
    if not ok:
        print(f'IK failed (error={err*1000:.1f} mm). Choose a reachable target.')
        return

    q_start = np.zeros(5)
    coeffs  = quintic_coeffs(q_start, q_end, T)

    print(f'IK solved  (error = {err*1000:.3f} mm)')
    print(f'Start EE   : {fk_position(q_start).round(4)}')
    print(f'Target EE  : {fk_position(q_end).round(4)}')
    print(f'q_end (°)  : {np.degrees(q_end).round(2)}')

    # Integrate the closed loop with a stiff solver
    t_span = (0.0, T * 1.5)                       # run 50% past trajectory end to see steady state
    t_eval = np.linspace(0, T * 1.5, 300)
    y0     = np.concatenate([q_start, np.zeros(5)])

    print(f'\nIntegrando dinâmica ({T*1.5:.1f} s, {len(t_eval)} pontos, solver LSODA)...')
    sol = solve_ivp(ode, t_span, y0, t_eval=t_eval,
                    args=(coeffs, T),
                    method='LSODA', rtol=1e-5, atol=1e-7)

    q_hist  = sol.y[:5].T
    qd_hist = sol.y[5:].T

    # Reference, reference velocity and control torques at each step
    q_des_hist  = np.zeros_like(q_hist)
    qd_des_hist = np.zeros_like(q_hist)
    tau_hist    = np.zeros_like(q_hist)
    for k, t in enumerate(t_eval):
        q_des, qd_des, _ = quintic_eval(coeffs, min(t, T))
        q_des_hist[k]  = q_des
        qd_des_hist[k] = qd_des
        tau_hist[k]    = control_law(q_hist[k], qd_hist[k], q_des, qd_des)

    # Final error
    e_final = q_hist[-1] - q_end
    ee_final_err = np.linalg.norm(fk_position(q_hist[-1]) - fk_position(q_end)) * 1000
    print(f'\nErro final de junta: {np.degrees(e_final[:3]).round(3)} ° | '
          f'{e_final[3]*1000:.3f} mm | {np.degrees(e_final[4]):.3f} °')
    print(f'Erro cartesiano final: {ee_final_err:.3f} mm')

    # Save plots (the four required + tracking + summary)
    print(f'\nSalvando gráficos em {args.output}/')
    plot_tracking(t_eval, q_hist, q_des_hist, args.output)
    plot_velocity(t_eval, qd_hist, qd_des_hist, args.output)
    plot_error(t_eval, q_hist, q_des_hist, args.output)
    plot_torques(t_eval, tau_hist, args.output)
    plot_summary(t_eval, q_hist, q_des_hist, qd_hist, tau_hist, q_start, q_end, args.output)

    print('\nFeito!')


if __name__ == '__main__':
    main()
