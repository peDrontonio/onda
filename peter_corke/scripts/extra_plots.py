#!/usr/bin/env python3
"""
Extra analysis plots for the Braco RRRPR manipulator (Robotics Toolbox model).

Generates a set of "nice to have" figures beyond the four required ones, useful
for the report's Resultados / Discussão sections:

    1. workspace_cloud.png     — reachable workspace (Monte-Carlo) + trajectory
    2. ee_path_3d.png          — 3-D end-effector path (home -> target)
    3. trajectory_profile.png  — quintic pos / vel / acc / jerk (C² continuity)
    4. gain_sweep_kp.png       — effect of Kp scaling on the closed-loop response
    5. pd_vs_pdg.png           — PD alone vs PD + gravity compensation
    6. torque_decomposition.png— PD term vs gravity term of the control torque
    7. error_convergence.png   — Cartesian error vs time (log scale)
    8. manipulability.png      — Yoshikawa index along the trajectory

Run (inside the project venv):
    python extra_plots.py [x] [y] [z] [--duration T] [--output DIR]
"""

import argparse
import os
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from braco_rtb import (
    robot, ik, fk_position, chain_points, gravload, plant_accel, JOINT_LIMITS,
)
from closed_loop_sim import quintic_coeffs, quintic_eval, KP, KD


# ── Colour-blind-safe categorical palette (Okabe–Ito), one hue per joint ──────
COLORS = ['#0072B2', '#D55E00', '#009E73', '#E69F00', '#CC79A7']
JOINT_LABELS = ['θ₁ (base)', 'θ₂', 'θ₃', 'd₄ (prism)', 'θ₅']
GRID = dict(alpha=0.3, linewidth=0.6)


def _style(ax, title=None, xl=None, yl=None):
    if title: ax.set_title(title, fontsize=11, fontweight='bold')
    if xl: ax.set_xlabel(xl, fontsize=10)
    if yl: ax.set_ylabel(yl, fontsize=10)
    ax.grid(True, **GRID)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)


def _disp(i, v):
    """Revolute -> degrees, prismatic (idx 3) -> mm."""
    return v * 1000 if i == 3 else np.degrees(v)


# ── Closed-loop simulation runner (variable gains / gravity comp) ─────────────

def simulate(q_start, q_end, T, kp, kd, gravity_comp=True, t_end=None, n=400):
    """Integrate the closed loop; return (t, q, qd, tau)."""
    t_end = t_end or T * 1.6
    coeffs = quintic_coeffs(q_start, q_end, T)

    def ctrl(q, qd, qdes, qddes):
        tau = kp @ (qdes - q) + kd @ (qddes - qd)
        if gravity_comp:
            tau = tau + gravload(q)
        return tau

    def ode(t, s):
        q, qd = s[:5], s[5:]
        qdes, qddes, _ = quintic_eval(coeffs, min(t, T))
        return np.concatenate([qd, plant_accel(q, qd, ctrl(q, qd, qdes, qddes))])

    t_eval = np.linspace(0, t_end, n)
    y0 = np.concatenate([q_start, np.zeros(5)])
    sol = solve_ivp(ode, (0, t_end), y0, t_eval=t_eval,
                    method='LSODA', rtol=1e-5, atol=1e-7)
    q_hist, qd_hist = sol.y[:5].T, sol.y[5:].T
    tau_hist = np.zeros_like(q_hist)
    for k, t in enumerate(t_eval):
        qdes, qddes, _ = quintic_eval(coeffs, min(t, T))
        tau_hist[k] = ctrl(q_hist[k], qd_hist[k], qdes, qddes)
    return t_eval, q_hist, qd_hist, tau_hist


# ── 1. Reachable workspace cloud ──────────────────────────────────────────────

def plot_workspace(q_start, q_end, out, n_samples=6000):
    rng = np.random.default_rng(0)
    lo = np.array([l for l, _ in JOINT_LIMITS])
    hi = np.array([h for _, h in JOINT_LIMITS])
    qs = lo + rng.random((n_samples, 5)) * (hi - lo)
    pts = np.array([fk_position(q) for q in qs])

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection='3d')
    sc = ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=pts[:, 2],
                    cmap='viridis', s=4, alpha=0.25)
    # arm at home & target
    for q, c, lbl in ((q_start, '#555555', 'home'), (q_end, '#D55E00', 'alvo')):
        xc, yc, zc = chain_points(q)
        ax.plot(xc, yc, zc, 'o-', color=c, linewidth=3, markersize=5, label=f'braço ({lbl})')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('Espaço de trabalho alcançável (Monte-Carlo, 6000 amostras)',
                 fontsize=12, fontweight='bold')
    fig.colorbar(sc, ax=ax, shrink=0.6, label='altura Z (m)', pad=0.1)
    ax.legend(loc='upper left', fontsize=9)
    ax.view_init(elev=22, azim=-60)
    path = os.path.join(out, 'workspace_cloud.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close()
    print(f'  ✓ {path}')


# ── 2. End-effector 3-D path ──────────────────────────────────────────────────

def plot_ee_path(t, q_hist, q_end, out):
    ee = np.array([fk_position(q) for q in q_hist])
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    # colour path by time
    for k in range(len(ee) - 1):
        ax.plot(ee[k:k+2, 0], ee[k:k+2, 1], ee[k:k+2, 2],
                color=plt.cm.plasma(k / len(ee)), linewidth=2.5)
    ax.scatter(*ee[0],  color='#009E73', s=90, label='início (home)')
    ax.scatter(*ee[-1], color='#D55E00', s=90, marker='*', label='alvo atingido')
    tgt = fk_position(q_end)
    ax.scatter(*tgt, color='k', s=60, marker='x', label='alvo (IK)')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('Caminho 3-D do efetuador (cor = tempo)', fontsize=12, fontweight='bold')
    ax.legend(fontsize=9)
    ax.view_init(elev=25, azim=-70)
    path = os.path.join(out, 'ee_path_3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close()
    print(f'  ✓ {path}')


# ── 3. Quintic profile: pos / vel / acc / jerk ────────────────────────────────

def plot_trajectory_profile(q_start, q_end, T, out):
    coeffs = quintic_coeffs(q_start, q_end, T)
    ts = np.linspace(0, T, 300)
    pos = np.array([quintic_eval(coeffs, t)[0] for t in ts])
    vel = np.array([quintic_eval(coeffs, t)[1] for t in ts])
    acc = np.array([quintic_eval(coeffs, t)[2] for t in ts])
    # jerk = d(acc)/dt numerically
    jerk = np.gradient(acc, ts, axis=0)

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle('Perfil da trajetória — polinômio quíntico (C² contínuo)',
                 fontsize=13, fontweight='bold')
    data = [(pos, 'Posição', '° / mm'), (vel, 'Velocidade', '°/s / mm/s'),
            (acc, 'Aceleração', '°/s² / mm/s²'), (jerk, 'Jerk', '°/s³ / mm/s³')]
    for ax, (d, name, unit) in zip(axes.flat, data):
        for i in range(5):
            ax.plot(ts, [_disp(i, v) for v in d[:, i]], color=COLORS[i],
                    linewidth=2, label=JOINT_LABELS[i])
        _style(ax, name, 'Tempo (s)', unit)
    axes[0, 0].legend(fontsize=8, ncol=2)
    plt.tight_layout()
    path = os.path.join(out, 'trajectory_profile.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close()
    print(f'  ✓ {path}')


# ── 4. Gain sweep (effect of Kp) ──────────────────────────────────────────────

def plot_gain_sweep(q_start, q_end, T, out, joint=2):
    scales = [0.25, 0.5, 1.0, 2.0, 4.0]
    cmap = plt.cm.viridis(np.linspace(0.1, 0.9, len(scales)))
    fig, ax = plt.subplots(figsize=(11, 6))
    coeffs = quintic_coeffs(q_start, q_end, T)
    ref = np.array([_disp(joint, quintic_eval(coeffs, min(t, T))[0][joint])
                    for t in np.linspace(0, T * 1.6, 400)])
    ax.plot(np.linspace(0, T * 1.6, 400), ref, 'k--', linewidth=1.8,
            label='referência q_des', zorder=5)
    for s, c in zip(scales, cmap):
        t, q, _, _ = simulate(q_start, q_end, T, s * KP, np.sqrt(s) * KD)
        ax.plot(t, [_disp(joint, v) for v in q[:, joint]], color=c,
                linewidth=2, label=f'Kp × {s:g}')
    _style(ax, f'Efeito do ganho Kp na resposta — junta {JOINT_LABELS[joint]}',
           'Tempo (s)', 'Posição (°)')
    ax.legend(fontsize=9, title='ganho proporcional')
    plt.tight_layout()
    path = os.path.join(out, 'gain_sweep_kp.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close()
    print(f'  ✓ {path}')


# ── 5. PD vs PD + gravity compensation ────────────────────────────────────────

def plot_pd_vs_pdg(q_start, q_end, T, out):
    fig, ax = plt.subplots(figsize=(11, 6))
    for gc, c, lbl in ((False, '#D55E00', 'PD puro'),
                       (True,  '#0072B2', 'PD + compensação de gravidade')):
        t, q, _, _ = simulate(q_start, q_end, T, KP, KD, gravity_comp=gc)
        ee = np.array([fk_position(v) for v in q])
        des = fk_position(q_end)
        err = np.linalg.norm(ee - des, axis=1) * 1000
        ax.plot(t, err, color=c, linewidth=2.2, label=lbl)
    _style(ax, 'Erro cartesiano: PD puro vs PD + gravidade',
           'Tempo (s)', 'Erro do efetuador (mm)')
    ax.legend(fontsize=10)
    plt.tight_layout()
    path = os.path.join(out, 'pd_vs_pdg.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close()
    print(f'  ✓ {path}')


# ── 6. Torque decomposition (PD term vs gravity term) ─────────────────────────

def plot_torque_decomposition(q_start, q_end, T, out):
    coeffs = quintic_coeffs(q_start, q_end, T)
    t, q, qd, _ = simulate(q_start, q_end, T, KP, KD)
    pd_term = np.zeros_like(q)
    g_term = np.zeros_like(q)
    for k, tk in enumerate(t):
        qdes, qddes, _ = quintic_eval(coeffs, min(tk, T))
        pd_term[k] = KP @ (qdes - q[k]) + KD @ (qddes - qd[k])
        g_term[k] = gravload(q[k])

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('Decomposição do torque de controle  τ = τ_PD + G(q)',
                 fontsize=13, fontweight='bold')
    for row, (data, name) in enumerate(
            [(pd_term, 'Termo PD  (Kp·e + Kd·ė)'),
             (g_term, 'Termo gravidade  G(q)'),
             (pd_term + g_term, 'Torque total  τ')]):
        ax = axes[row]
        for i in range(5):
            ax.plot(t, data[:, i], color=COLORS[i], linewidth=1.8, label=JOINT_LABELS[i])
        ax.axhline(0, color='gray', linewidth=0.8, linestyle='--')
        _style(ax, name, None, 'N·m / N')
    axes[0].legend(fontsize=8, ncol=5, loc='upper right')
    axes[-1].set_xlabel('Tempo (s)', fontsize=10)
    plt.tight_layout()
    path = os.path.join(out, 'torque_decomposition.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close()
    print(f'  ✓ {path}')


# ── 7. Error convergence (log scale) ──────────────────────────────────────────

def plot_error_convergence(q_start, q_end, T, out):
    t, q, _, _ = simulate(q_start, q_end, T, KP, KD)
    ee = np.array([fk_position(v) for v in q])
    des = fk_position(q_end)
    err = np.linalg.norm(ee - des, axis=1) * 1000
    err = np.clip(err, 1e-4, None)

    fig, ax = plt.subplots(figsize=(11, 6))
    ax.semilogy(t, err, color='#0072B2', linewidth=2.2)
    for tol, c, lbl in ((1.0, '#E69F00', '1 mm'), (0.1, '#009E73', '0,1 mm')):
        ax.axhline(tol, color=c, linewidth=1.2, linestyle='--', label=f'tolerância {lbl}')
        idx = np.argmax(err < tol) if np.any(err < tol) else None
        if idx:
            ax.scatter(t[idx], err[idx], color=c, s=45, zorder=5)
    _style(ax, 'Convergência do erro cartesiano (escala log)',
           'Tempo (s)', 'Erro do efetuador (mm)')
    ax.legend(fontsize=9)
    plt.tight_layout()
    path = os.path.join(out, 'error_convergence.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close()
    print(f'  ✓ {path}')


# ── 8. Manipulability (Yoshikawa) along the trajectory ────────────────────────

def plot_manipulability(q_start, q_end, T, out):
    t, q, _, _ = simulate(q_start, q_end, T, KP, KD)
    w = []
    for qk in q:
        J = robot.jacob0(qk)[:3]           # translational part
        w.append(float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0))))
    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(t, w, color='#CC79A7', linewidth=2.2)
    ax.fill_between(t, w, color='#CC79A7', alpha=0.15)
    _style(ax, 'Manipulabilidade de Yoshikawa ao longo do movimento',
           'Tempo (s)', 'w(q) = √det(J·Jᵀ)')
    plt.tight_layout()
    path = os.path.join(out, 'manipulability.png')
    plt.savefig(path, dpi=150, bbox_inches='tight'); plt.close()
    print(f'  ✓ {path}')


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description='Extra analysis plots (Braco RRRPR, RTB)')
    p.add_argument('x', type=float, nargs='?', default=-0.155)
    p.add_argument('y', type=float, nargs='?', default=0.290)
    p.add_argument('z', type=float, nargs='?', default=0.361)
    p.add_argument('--duration', '-d', type=float, default=4.0)
    p.add_argument('--output', '-o', default='output')
    args = p.parse_args()

    os.makedirs(args.output, exist_ok=True)
    q_end, ok, err = ik((args.x, args.y, args.z))
    if not ok:
        print(f'IK falhou (erro={err*1000:.1f} mm). Escolha um alvo alcançável.')
        return
    q_start = np.zeros(5)
    T = args.duration
    print(f'IK ok (erro {err*1000:.3f} mm). Gerando gráficos em {args.output}/')

    plot_workspace(q_start, q_end, args.output)
    t, q, qd, tau = simulate(q_start, q_end, T, KP, KD)
    plot_ee_path(t, q, q_end, args.output)
    plot_trajectory_profile(q_start, q_end, T, args.output)
    plot_gain_sweep(q_start, q_end, T, args.output)
    plot_pd_vs_pdg(q_start, q_end, T, args.output)
    plot_torque_decomposition(q_start, q_end, T, args.output)
    plot_error_convergence(q_start, q_end, T, args.output)
    plot_manipulability(q_start, q_end, T, args.output)
    print('\nFeito!')


if __name__ == '__main__':
    main()
