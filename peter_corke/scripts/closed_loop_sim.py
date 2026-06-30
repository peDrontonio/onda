#!/usr/bin/env python3
"""
Closed-loop forward-dynamics simulation of the Braco RRRPR manipulator.

Control law: τ = Kp*(q_des − q) − Kd*q̇ + G(q)   [PD + gravity compensation]

The plant integrates the full nonlinear dynamics:
    q̈ = M(q)⁻¹ · [τ − C(q,q̇) − G(q)]

Reference is a quintic polynomial trajectory from home (q=0) to a target pose
computed via IK.

Run:
    python closed_loop_sim.py [x] [y] [z] [--duration T]

Saves plots to:  output/
"""

import argparse
import os
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from cinematica_inversa import ik, fk_position, JOINT_NAMES


# ── Dynamics model (masses from URDF) ─────────────────────────────────────────

MASSES = [1.910661, 6.846799, 7.274379, 14.513248, 4.193506, 0.654538]  # kg
G_ACC  = 9.81  # m/s²

# Diagonal inertia (simplified — principal axes from URDF ixx/iyy/izz)
INERTIA_DIAG = [0.002136, 0.014211, 0.016500, 0.002000, 0.001000]


def inertia_matrix(q):
    M = np.diag(INERTIA_DIAG)
    c3 = np.cos(q[2])
    M[1, 2] = 0.005 * c3
    M[2, 1] = M[1, 2]
    return M


def coriolis(q, qd):
    C = np.zeros(5)
    s2 = np.sin(q[1])
    C[0] = -0.01 * qd[1] * s2 * qd[0]
    C[1] =  0.01 * qd[0] ** 2 * s2
    return C


def gravity(q):
    G = np.zeros(5)
    m2, m3, m4 = MASSES[2], MASSES[3], MASSES[4]
    c2  = np.cos(q[1])
    c23 = np.cos(q[1] + q[2])
    s23 = np.sin(q[1] + q[2])
    G[1] = (m2 + m3 + m4) * G_ACC * 0.1  * c2
    G[2] = (m3 + m4)      * G_ACC * 0.08 * c23
    G[3] =  m4             * G_ACC        * s23
    G[4] = 0.01            * G_ACC * np.cos(q[4])
    return G


# ── PD + gravity compensation gains ───────────────────────────────────────────

KP = np.diag([120.0, 100.0, 80.0, 500.0, 30.0])
KD = np.diag([ 10.0,   8.0,  6.0,  30.0,  2.0])


def control_law(q, qd, q_des, qd_des):
    e  = q_des - q
    ed = qd_des - qd
    return KP @ e + KD @ ed + gravity(q)


# ── Quintic polynomial reference ───────────────────────────────────────────────

def quintic_coeffs(q0, qf, T):
    """Return 6 coefficients a0..a5 for each joint (shape 5×6)."""
    dq = qf - q0
    T2, T3, T4, T5 = T**2, T**3, T**4, T**5
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


# ── ODE ───────────────────────────────────────────────────────────────────────

def ode(t, state, coeffs, T):
    q  = state[:5]
    qd = state[5:]

    if t < T:
        q_des, qd_des, _ = quintic_eval(coeffs, t)
    else:
        q_des, qd_des, _ = quintic_eval(coeffs, T)

    tau = control_law(q, qd, q_des, qd_des)
    M   = inertia_matrix(q)
    C   = coriolis(q, qd)
    Gq  = gravity(q)
    qdd = np.linalg.solve(M, tau - C - Gq)

    return np.concatenate([qd, qdd])


# ── Plots ─────────────────────────────────────────────────────────────────────

JOINT_LABELS = ['θ₁ (base_rot1)', 'θ₂ (rot1_rot2)', 'θ₃ (rot2_rot3)',
                'd₄ (prism, mm)', 'θ₅ (prism1_rot4)']
COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']


def _q_to_display(i, val):
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


def plot_summary(t_eval, q_hist, q_des_hist, tau_hist, q_start, q_end, out):
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Simulação em malha fechada — Braço RRRPR', fontsize=14, fontweight='bold')

    # Positions
    ax = axes[0, 0]
    for i in range(5):
        disp = [_q_to_display(i, v) for v in q_hist[:, i]]
        des  = [_q_to_display(i, v) for v in q_des_hist[:, i]]
        ax.plot(t_eval, des, '--', color=COLORS[i], linewidth=1.2)
        ax.plot(t_eval, disp, '-', color=COLORS[i], linewidth=2, label=JOINT_LABELS[i].split('(')[0].strip())
    ax.set_title('Posições (— real,  -- referência)')
    ax.set_xlabel('Tempo (s)')
    ax.set_ylabel('Posição')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Errors
    ax = axes[0, 1]
    for i in range(5):
        err = [_q_to_display(i, q_des_hist[k, i] - q_hist[k, i]) for k in range(len(t_eval))]
        ax.plot(t_eval, err, color=COLORS[i], linewidth=2, label=JOINT_LABELS[i].split('(')[0].strip())
    ax.axhline(0, color='gray', linewidth=0.8, linestyle='--')
    ax.set_title('Erro de rastreamento e(t)')
    ax.set_xlabel('Tempo (s)')
    ax.set_ylabel('Erro (° ou mm)')
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

    # End-effector tracking
    ax = axes[1, 1]
    ee_real = np.array([fk_position(q_hist[k]) for k in range(len(t_eval))])
    # Compute desired EE from q_des
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
    parser = argparse.ArgumentParser(description='Closed-loop PD+G simulation of Braco')
    parser.add_argument('x', type=float, nargs='?', default=-0.155,
                        help='Target X (m)')
    parser.add_argument('y', type=float, nargs='?', default=0.290,
                        help='Target Y (m)')
    parser.add_argument('z', type=float, nargs='?', default=0.361,
                        help='Target Z (m)')
    parser.add_argument('--duration', '-d', type=float, default=4.0,
                        help='Trajectory duration (s)')
    parser.add_argument('--output', '-o', default='output',
                        help='Output directory for plots')
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)
    T = args.duration

    # IK
    q_end, ok, err = ik((args.x, args.y, args.z))
    if not ok:
        print(f'IK failed (error={err*1000:.1f} mm). Choose a reachable target.')
        return

    q_start = np.zeros(5)
    coeffs  = quintic_coeffs(q_start, q_end, T)

    p_start = fk_position(q_start)
    p_end   = fk_position(q_end)
    print(f'IK solved  (error = {err*1000:.3f} mm)')
    print(f'Start EE   : {p_start.round(4)}')
    print(f'Target EE  : {p_end.round(4)}')
    print(f'q_end (°)  : {np.degrees(q_end).round(2)}')

    # Integrate
    t_span = (0.0, T * 1.5)           # run 50% past trajectory end to see steady state
    t_eval = np.linspace(0, T * 1.5, 600)
    y0     = np.concatenate([q_start, np.zeros(5)])

    print(f'\nIntegrando dinâmica ({T*1.5:.1f} s, {len(t_eval)} pontos)...')
    sol = solve_ivp(ode, t_span, y0, t_eval=t_eval,
                    args=(coeffs, T),
                    method='RK45', rtol=1e-6, atol=1e-8,
                    max_step=0.002)

    q_hist = sol.y[:5].T
    qd_hist = sol.y[5:].T

    # Reference and torques at each time step
    q_des_hist  = np.zeros_like(q_hist)
    qd_des_hist = np.zeros_like(q_hist)
    tau_hist    = np.zeros_like(q_hist)
    for k, t in enumerate(t_eval):
        t_clamped  = min(t, T)
        qd_k, qddk, _ = quintic_eval(coeffs, t_clamped)
        q_des_hist[k]  = qd_k
        qd_des_hist[k] = qddk
        tau_hist[k]    = control_law(q_hist[k], qd_hist[k], qd_k, qddk)

    # Final error
    e_final = q_hist[-1] - q_end
    ee_final_err = np.linalg.norm(fk_position(q_hist[-1]) - fk_position(q_end)) * 1000
    print(f'\nErro final de junta: {np.degrees(e_final[:3]).round(3)} ° | '
          f'{e_final[3]*1000:.3f} mm | {np.degrees(e_final[4]):.3f} °')
    print(f'Erro cartesiano final: {ee_final_err:.3f} mm')

    # Save plots
    print(f'\nSalvando gráficos em {args.output}/')
    plot_tracking(t_eval, q_hist, q_des_hist, args.output)
    plot_error(t_eval, q_hist, q_des_hist, args.output)
    plot_torques(t_eval, tau_hist, args.output)
    plot_summary(t_eval, q_hist, q_des_hist, tau_hist, q_start, q_end, args.output)

    print('\nFeito!')


if __name__ == '__main__':
    main()
