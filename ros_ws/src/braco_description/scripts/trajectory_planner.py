#!/usr/bin/env python3
"""
Advanced Trajectory Planner for Braco Manipulator

This script implements:
1. Quintic (5th degree) polynomial trajectory planning
   - Ensures continuous position, velocity, and acceleration
   - Zero velocity and acceleration at start/end (no infinite jerk)
2. Dynamic analysis with torque estimation
3. Real-time plotting of position, velocity, acceleration, and torque

The quintic polynomial ensures:
- Smooth start and stop (zero velocity at boundaries)
- Zero acceleration at boundaries (no infinite jerk)
- Continuous jerk profile

Usage:
    ros2 run braco_description trajectory_planner.py <x> <y> <z> --duration 3.0

Author: Pedro Antonio
Date: 2025-12-16
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import argparse
from datetime import datetime


class QuinticTrajectoryPlanner:
    """
    Quintic (5th degree) polynomial trajectory planner.
    
    The quintic polynomial is: q(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
    
    This ensures:
    - Position continuity (C0)
    - Velocity continuity (C1) 
    - Acceleration continuity (C2)
    - Finite jerk at all times
    
    Boundary conditions:
    - q(0) = q0, q(T) = qf
    - q'(0) = v0, q'(T) = vf
    - q''(0) = a0, q''(T) = af
    """
    
    def __init__(self):
        pass
    
    def compute_coefficients(self, q0, qf, T, v0=0, vf=0, a0=0, af=0):
        """
        Compute quintic polynomial coefficients.
        
        Args:
            q0: Initial position
            qf: Final position
            T: Duration
            v0: Initial velocity (default 0)
            vf: Final velocity (default 0)
            a0: Initial acceleration (default 0)
            af: Final acceleration (default 0)
        
        Returns:
            Coefficients [a0, a1, a2, a3, a4, a5]
        """
        # Quintic polynomial coefficients derived from boundary conditions
        a0_coef = q0
        a1_coef = v0
        a2_coef = a0 / 2
        
        # Solving the system of equations for a3, a4, a5
        T2 = T * T
        T3 = T2 * T
        T4 = T3 * T
        T5 = T4 * T
        
        # From boundary conditions at t=T
        a3_coef = (20 * (qf - q0) - (8 * vf + 12 * v0) * T - (3 * a0 - af) * T2) / (2 * T3)
        a4_coef = (30 * (q0 - qf) + (14 * vf + 16 * v0) * T + (3 * a0 - 2 * af) * T2) / (2 * T4)
        a5_coef = (12 * (qf - q0) - 6 * (vf + v0) * T - (a0 - af) * T2) / (2 * T5)
        
        return np.array([a0_coef, a1_coef, a2_coef, a3_coef, a4_coef, a5_coef])
    
    def evaluate(self, coeffs, t):
        """
        Evaluate position at time t.
        """
        return (coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 + 
                coeffs[3]*t**3 + coeffs[4]*t**4 + coeffs[5]*t**5)
    
    def evaluate_velocity(self, coeffs, t):
        """
        Evaluate velocity at time t (first derivative).
        """
        return (coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*t**2 + 
                4*coeffs[4]*t**3 + 5*coeffs[5]*t**4)
    
    def evaluate_acceleration(self, coeffs, t):
        """
        Evaluate acceleration at time t (second derivative).
        """
        return (2*coeffs[2] + 6*coeffs[3]*t + 12*coeffs[4]*t**2 + 20*coeffs[5]*t**3)
    
    def evaluate_jerk(self, coeffs, t):
        """
        Evaluate jerk at time t (third derivative).
        """
        return (6*coeffs[3] + 24*coeffs[4]*t + 60*coeffs[5]*t**2)
    
    def plan_trajectory(self, q_start, q_end, duration, num_points=100):
        """
        Plan a complete trajectory for all joints.
        
        Args:
            q_start: Initial joint positions [q1, q2, q3, q4, q5]
            q_end: Final joint positions [q1, q2, q3, q4, q5]
            duration: Total trajectory duration in seconds
            num_points: Number of trajectory points
        
        Returns:
            Dictionary with time, positions, velocities, accelerations, jerk for each joint
        """
        num_joints = len(q_start)
        t = np.linspace(0, duration, num_points)
        
        positions = np.zeros((num_joints, num_points))
        velocities = np.zeros((num_joints, num_points))
        accelerations = np.zeros((num_joints, num_points))
        jerks = np.zeros((num_joints, num_points))
        
        coefficients = []
        
        for j in range(num_joints):
            coeffs = self.compute_coefficients(q_start[j], q_end[j], duration)
            coefficients.append(coeffs)
            
            for i, ti in enumerate(t):
                positions[j, i] = self.evaluate(coeffs, ti)
                velocities[j, i] = self.evaluate_velocity(coeffs, ti)
                accelerations[j, i] = self.evaluate_acceleration(coeffs, ti)
                jerks[j, i] = self.evaluate_jerk(coeffs, ti)
        
        return {
            'time': t,
            'positions': positions,
            'velocities': velocities,
            'accelerations': accelerations,
            'jerks': jerks,
            'coefficients': coefficients
        }


class BracoDynamics:
    """
    Simplified dynamics model for the Braco manipulator.
    Calculates torques based on inertia, Coriolis, and gravity terms.
    """
    
    def __init__(self):
        # Link masses (approximate from URDF inertias)
        self.masses = [1.91, 6.85, 7.27, 14.51, 4.19, 0.65]  # kg
        
        # Link lengths
        self.link_lengths = [0.035, 0.1, 0.08, 0.18, 0.165, 0.1]  # m
        
        # Approximate moments of inertia (kg·m²)
        self.inertias = [
            0.002,   # Joint 1
            0.014,   # Joint 2
            0.017,   # Joint 3
            0.002,   # Joint 4 (prismatic - effective mass)
            0.001    # Joint 5
        ]
        
        # Gravity
        self.g = 9.81
        
        # Motor/gear parameters (approximate)
        self.gear_ratios = [100, 100, 100, 50, 50]
        self.motor_inertias = [0.0001, 0.0001, 0.0001, 0.0001, 0.0001]
        
    def compute_inertia_matrix(self, q):
        """
        Compute simplified inertia matrix M(q).
        """
        n = len(q)
        M = np.diag(self.inertias[:n])
        
        # Add configuration-dependent terms (simplified)
        c2 = np.cos(q[1]) if len(q) > 1 else 1
        c3 = np.cos(q[2]) if len(q) > 2 else 1
        
        # Cross-coupling terms
        if n >= 3:
            M[1, 2] = 0.005 * c3
            M[2, 1] = M[1, 2]
        
        return M
    
    def compute_coriolis(self, q, qd):
        """
        Compute simplified Coriolis/centrifugal terms C(q, qd).
        """
        n = len(q)
        C = np.zeros(n)
        
        if n >= 2:
            s2 = np.sin(q[1])
            C[0] = -0.01 * qd[1] * s2 * qd[0]
            C[1] = 0.01 * qd[0]**2 * s2
        
        return C
    
    def compute_gravity(self, q):
        """
        Compute gravity torques G(q).
        """
        n = len(q)
        G = np.zeros(n)
        
        # Joint 1: No gravity effect (vertical rotation)
        G[0] = 0
        
        # Joint 2: Gravity acts on arm
        if n >= 2:
            c2 = np.cos(q[1])
            G[1] = (self.masses[2] + self.masses[3] + self.masses[4]) * self.g * 0.1 * c2
        
        # Joint 3: Gravity on forearm
        if n >= 3:
            c23 = np.cos(q[1] + q[2])
            G[2] = (self.masses[3] + self.masses[4]) * self.g * 0.08 * c23
        
        # Joint 4: Prismatic - gravity component
        if n >= 4:
            s23 = np.sin(q[1] + q[2])
            G[3] = self.masses[4] * self.g * s23
        
        # Joint 5: Small gravity effect
        if n >= 5:
            G[4] = 0.01 * self.g * np.cos(q[4])
        
        return G
    
    def compute_torques(self, q, qd, qdd):
        """
        Compute joint torques using inverse dynamics.
        τ = M(q)q̈ + C(q,q̇) + G(q)
        
        Args:
            q: Joint positions
            qd: Joint velocities
            qdd: Joint accelerations
        
        Returns:
            Joint torques
        """
        M = self.compute_inertia_matrix(q)
        C = self.compute_coriolis(q, qd)
        G = self.compute_gravity(q)
        
        tau = M @ qdd + C + G
        return tau


class TrajectoryVisualizer:
    """
    Creates and saves trajectory visualization plots.
    """
    
    def __init__(self, output_dir='output'):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        self.joint_names = [
            'base_rot1 (θ₁)',
            'rot1_rot2 (θ₂)',
            'rot2_rot3 (θ₃)',
            'rot3_prism1 (d₄)',
            'prism1_rot4 (θ₅)'
        ]
        
        self.colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
        
    def plot_positions(self, trajectory_data, filename='positions.png'):
        """Plot joint positions over time."""
        fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=True)
        fig.suptitle('Posições das Juntas (Polinômio Quíntico)', fontsize=14, fontweight='bold')
        
        t = trajectory_data['time']
        positions = trajectory_data['positions']
        
        for i, (ax, name, color) in enumerate(zip(axes, self.joint_names, self.colors)):
            if i == 3:  # Prismatic joint
                ax.plot(t, positions[i] * 1000, color=color, linewidth=2)
                ax.set_ylabel('Posição (mm)', fontsize=10)
            else:
                ax.plot(t, np.degrees(positions[i]), color=color, linewidth=2)
                ax.set_ylabel('Posição (°)', fontsize=10)
            
            ax.set_title(name, fontsize=11, loc='left')
            ax.grid(True, alpha=0.3)
            ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        
        axes[-1].set_xlabel('Tempo (s)', fontsize=11)
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, filename), dpi=150, bbox_inches='tight')
        plt.close()
        print(f'  ✓ {filename} salvo')
        
    def plot_velocities(self, trajectory_data, filename='velocities.png'):
        """Plot joint velocities over time."""
        fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=True)
        fig.suptitle('Velocidades das Juntas (Polinômio Quíntico)', fontsize=14, fontweight='bold')
        
        t = trajectory_data['time']
        velocities = trajectory_data['velocities']
        
        for i, (ax, name, color) in enumerate(zip(axes, self.joint_names, self.colors)):
            if i == 3:  # Prismatic joint
                ax.plot(t, velocities[i] * 1000, color=color, linewidth=2)
                ax.set_ylabel('Velocidade (mm/s)', fontsize=10)
            else:
                ax.plot(t, np.degrees(velocities[i]), color=color, linewidth=2)
                ax.set_ylabel('Velocidade (°/s)', fontsize=10)
            
            ax.set_title(name, fontsize=11, loc='left')
            ax.grid(True, alpha=0.3)
            ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
            
            # Mark zero velocity at start and end
            ax.scatter([t[0], t[-1]], [0, 0], color='red', s=50, zorder=5, label='v=0 (suave)')
        
        axes[0].legend(loc='upper right')
        axes[-1].set_xlabel('Tempo (s)', fontsize=11)
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, filename), dpi=150, bbox_inches='tight')
        plt.close()
        print(f'  ✓ {filename} salvo')
        
    def plot_accelerations(self, trajectory_data, filename='accelerations.png'):
        """Plot joint accelerations over time."""
        fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=True)
        fig.suptitle('Acelerações das Juntas (Polinômio Quíntico)', fontsize=14, fontweight='bold')
        
        t = trajectory_data['time']
        accelerations = trajectory_data['accelerations']
        
        for i, (ax, name, color) in enumerate(zip(axes, self.joint_names, self.colors)):
            if i == 3:  # Prismatic joint
                ax.plot(t, accelerations[i] * 1000, color=color, linewidth=2)
                ax.set_ylabel('Aceleração (mm/s²)', fontsize=10)
            else:
                ax.plot(t, np.degrees(accelerations[i]), color=color, linewidth=2)
                ax.set_ylabel('Aceleração (°/s²)', fontsize=10)
            
            ax.set_title(name, fontsize=11, loc='left')
            ax.grid(True, alpha=0.3)
            ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
            
            # Mark zero acceleration at start and end
            ax.scatter([t[0], t[-1]], [0, 0], color='green', s=50, zorder=5, label='a=0 (sem jerk infinito)')
        
        axes[0].legend(loc='upper right')
        axes[-1].set_xlabel('Tempo (s)', fontsize=11)
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, filename), dpi=150, bbox_inches='tight')
        plt.close()
        print(f'  ✓ {filename} salvo')
    
    def plot_jerks(self, trajectory_data, filename='jerks.png'):
        """Plot joint jerks over time."""
        fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=True)
        fig.suptitle('Jerk das Juntas (Derivada da Aceleração)', fontsize=14, fontweight='bold')
        
        t = trajectory_data['time']
        jerks = trajectory_data['jerks']
        
        for i, (ax, name, color) in enumerate(zip(axes, self.joint_names, self.colors)):
            if i == 3:  # Prismatic joint
                ax.plot(t, jerks[i] * 1000, color=color, linewidth=2)
                ax.set_ylabel('Jerk (mm/s³)', fontsize=10)
            else:
                ax.plot(t, np.degrees(jerks[i]), color=color, linewidth=2)
                ax.set_ylabel('Jerk (°/s³)', fontsize=10)
            
            ax.set_title(name, fontsize=11, loc='left')
            ax.grid(True, alpha=0.3)
            ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        
        axes[-1].set_xlabel('Tempo (s)', fontsize=11)
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, filename), dpi=150, bbox_inches='tight')
        plt.close()
        print(f'  ✓ {filename} salvo')
        
    def plot_torques(self, trajectory_data, torques, filename='torques.png'):
        """Plot joint torques over time."""
        fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=True)
        fig.suptitle('Torques das Juntas (τ = M(q)q̈ + C(q,q̇) + G(q))', fontsize=14, fontweight='bold')
        
        t = trajectory_data['time']
        
        units = ['N·m', 'N·m', 'N·m', 'N', 'N·m']  # Joint 4 is prismatic (force)
        
        for i, (ax, name, color, unit) in enumerate(zip(axes, self.joint_names, self.colors, units)):
            ax.plot(t, torques[i], color=color, linewidth=2)
            ax.set_ylabel(f'Torque ({unit})', fontsize=10)
            ax.set_title(name, fontsize=11, loc='left')
            ax.grid(True, alpha=0.3)
            ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
            
            # Show max torque
            max_tau = np.max(np.abs(torques[i]))
            ax.axhline(y=max_tau, color='red', linestyle=':', alpha=0.5, label=f'|τ|_max = {max_tau:.2f}')
            ax.axhline(y=-max_tau, color='red', linestyle=':', alpha=0.5)
            ax.legend(loc='upper right', fontsize=8)
        
        axes[-1].set_xlabel('Tempo (s)', fontsize=11)
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, filename), dpi=150, bbox_inches='tight')
        plt.close()
        print(f'  ✓ {filename} salvo')
    
    def plot_summary(self, trajectory_data, torques, filename='trajectory_summary.png'):
        """Create a summary plot with all data."""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle('Resumo da Trajetória - Polinômio Quíntico (Grau 5)', fontsize=14, fontweight='bold')
        
        t = trajectory_data['time']
        
        # Position plot
        ax = axes[0, 0]
        for i, (name, color) in enumerate(zip(self.joint_names, self.colors)):
            if i == 3:
                ax.plot(t, trajectory_data['positions'][i] * 1000, color=color, 
                       linewidth=2, label=f'{name} (mm)')
            else:
                ax.plot(t, np.degrees(trajectory_data['positions'][i]), color=color, 
                       linewidth=2, label=f'{name} (°)')
        ax.set_xlabel('Tempo (s)')
        ax.set_ylabel('Posição')
        ax.set_title('Posições das Juntas')
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)
        
        # Velocity plot
        ax = axes[0, 1]
        for i, (name, color) in enumerate(zip(self.joint_names, self.colors)):
            if i == 3:
                ax.plot(t, trajectory_data['velocities'][i] * 1000, color=color, linewidth=2)
            else:
                ax.plot(t, np.degrees(trajectory_data['velocities'][i]), color=color, linewidth=2)
        ax.set_xlabel('Tempo (s)')
        ax.set_ylabel('Velocidade')
        ax.set_title('Velocidades das Juntas (v₀=vf=0)')
        ax.grid(True, alpha=0.3)
        ax.scatter([t[0], t[-1]], [0, 0], color='red', s=100, zorder=5)
        
        # Acceleration plot
        ax = axes[1, 0]
        for i, (name, color) in enumerate(zip(self.joint_names, self.colors)):
            if i == 3:
                ax.plot(t, trajectory_data['accelerations'][i] * 1000, color=color, linewidth=2)
            else:
                ax.plot(t, np.degrees(trajectory_data['accelerations'][i]), color=color, linewidth=2)
        ax.set_xlabel('Tempo (s)')
        ax.set_ylabel('Aceleração')
        ax.set_title('Acelerações das Juntas (a₀=af=0 → sem jerk infinito)')
        ax.grid(True, alpha=0.3)
        ax.scatter([t[0], t[-1]], [0, 0], color='green', s=100, zorder=5)
        
        # Torque plot
        ax = axes[1, 1]
        for i, (name, color) in enumerate(zip(self.joint_names, self.colors)):
            ax.plot(t, torques[i], color=color, linewidth=2, label=name)
        ax.set_xlabel('Tempo (s)')
        ax.set_ylabel('Torque (N·m / N)')
        ax.set_title('Torques das Juntas')
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, filename), dpi=150, bbox_inches='tight')
        plt.close()
        print(f'  ✓ {filename} salvo')
    
    def generate_report(self, trajectory_data, torques, q_start, q_end, duration, target_pos):
        """Generate a text report about the trajectory."""
        report_path = os.path.join(self.output_dir, 'trajectory_report.txt')
        
        with open(report_path, 'w') as f:
            f.write("=" * 60 + "\n")
            f.write("RELATÓRIO DE PLANEJAMENTO DE TRAJETÓRIA\n")
            f.write("Braco Manipulator - Polinômio Quíntico (Grau 5)\n")
            f.write("=" * 60 + "\n\n")
            
            f.write(f"Data/Hora: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            f.write("PARÂMETROS DA TRAJETÓRIA:\n")
            f.write("-" * 40 + "\n")
            f.write(f"Posição alvo (Cartesiano): x={target_pos[0]:.3f}m, y={target_pos[1]:.3f}m, z={target_pos[2]:.3f}m\n")
            f.write(f"Duração: {duration:.2f} segundos\n")
            f.write(f"Pontos de amostragem: {len(trajectory_data['time'])}\n\n")
            
            f.write("TIPO DE INTERPOLAÇÃO:\n")
            f.write("-" * 40 + "\n")
            f.write("Polinômio Quíntico (5º grau):\n")
            f.write("  q(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵\n\n")
            f.write("Condições de contorno:\n")
            f.write("  - Posição inicial e final especificadas\n")
            f.write("  - Velocidade inicial e final = 0 (partida/parada suave)\n")
            f.write("  - Aceleração inicial e final = 0 (sem jerk infinito)\n\n")
            
            f.write("VANTAGENS DO POLINÔMIO QUÍNTICO:\n")
            f.write("-" * 40 + "\n")
            f.write("  ✓ Continuidade C² (posição, velocidade, aceleração)\n")
            f.write("  ✓ Jerk finito em todos os pontos\n")
            f.write("  ✓ Sem picos de aceleração infinita\n")
            f.write("  ✓ Movimento suave e contínuo\n")
            f.write("  ✓ Reduz desgaste mecânico\n")
            f.write("  ✓ Menor vibração estrutural\n\n")
            
            f.write("POSIÇÕES DAS JUNTAS:\n")
            f.write("-" * 40 + "\n")
            f.write(f"{'Junta':<20} {'Inicial':<15} {'Final':<15} {'Unidade'}\n")
            for i, name in enumerate(self.joint_names):
                if i == 3:
                    f.write(f"{name:<20} {q_start[i]*1000:>12.2f}   {q_end[i]*1000:>12.2f}   mm\n")
                else:
                    f.write(f"{name:<20} {np.degrees(q_start[i]):>12.2f}   {np.degrees(q_end[i]):>12.2f}   graus\n")
            
            f.write("\nANÁLISE DE VELOCIDADE:\n")
            f.write("-" * 40 + "\n")
            f.write(f"{'Junta':<20} {'Vel. Máx':<15} {'Unidade'}\n")
            for i, name in enumerate(self.joint_names):
                max_vel = np.max(np.abs(trajectory_data['velocities'][i]))
                if i == 3:
                    f.write(f"{name:<20} {max_vel*1000:>12.4f}   mm/s\n")
                else:
                    f.write(f"{name:<20} {np.degrees(max_vel):>12.4f}   °/s\n")
            
            f.write("\nANÁLISE DE ACELERAÇÃO:\n")
            f.write("-" * 40 + "\n")
            f.write(f"{'Junta':<20} {'Acel. Máx':<15} {'Unidade'}\n")
            for i, name in enumerate(self.joint_names):
                max_acc = np.max(np.abs(trajectory_data['accelerations'][i]))
                if i == 3:
                    f.write(f"{name:<20} {max_acc*1000:>12.4f}   mm/s²\n")
                else:
                    f.write(f"{name:<20} {np.degrees(max_acc):>12.4f}   °/s²\n")
            
            f.write("\nANÁLISE DE TORQUE:\n")
            f.write("-" * 40 + "\n")
            f.write(f"{'Junta':<20} {'Torque Máx':<15} {'Unidade'}\n")
            units = ['N·m', 'N·m', 'N·m', 'N', 'N·m']
            for i, (name, unit) in enumerate(zip(self.joint_names, units)):
                max_tau = np.max(np.abs(torques[i]))
                f.write(f"{name:<20} {max_tau:>12.4f}   {unit}\n")
            
            f.write("\n" + "=" * 60 + "\n")
            f.write("Arquivos gerados na pasta 'output/':\n")
            f.write("  - positions.png\n")
            f.write("  - velocities.png\n")
            f.write("  - accelerations.png\n")
            f.write("  - jerks.png\n")
            f.write("  - torques.png\n")
            f.write("  - trajectory_summary.png\n")
            f.write("  - trajectory_report.txt\n")
            f.write("=" * 60 + "\n")
        
        print(f'  ✓ trajectory_report.txt salvo')


class BracoKinematics:
    """Kinematics calculator for the Braco RRRPR manipulator."""
    
    def __init__(self):
        self.d1 = 0.035
        self.d2 = 0.065
        self.a1 = 0.1
        self.a2 = 0.08
        self.d3 = 0.1
        self.a3 = -0.18
        self.d4 = 0.1
        self.d5 = 0.165
        self.base_height = self.d1 + self.d2
        
        self.joint_limits = {
            'base_rot1': (0.0, 6.283185),
            'rot1_rot2': (0.0, 6.283185),
            'rot2_rot3': (0.0, 6.283185),
            'rot3_prism1': (0.0, 0.1),
            'prism1_rot4': (-1.047198, 1.047198)
        }
    
    def inverse_kinematics(self, x, y, z, q5=0.0):
        """Calculate joint positions to reach target Cartesian position."""
        q1 = np.arctan2(y, x)
        if q1 < 0:
            q1 += 2 * np.pi
        
        r = np.sqrt(x**2 + y**2)
        z_eff = z - self.base_height
        
        L1 = np.sqrt(self.a2**2 + self.d3**2)
        L2_base = np.sqrt(self.a3**2 + self.d4**2)
        
        target_dist = np.sqrt(r**2 + z_eff**2)
        d4 = 0.05
        L2 = L2_base + self.d5 + d4
        
        cos_q3 = (target_dist**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_q3 = np.clip(cos_q3, -1, 1)
        q3 = np.arccos(cos_q3)
        
        alpha = np.arctan2(z_eff, r)
        beta = np.arctan2(L2 * np.sin(q3), L1 + L2 * np.cos(q3))
        q2 = alpha + beta
        
        while q2 < 0:
            q2 += 2 * np.pi
        while q3 < 0:
            q3 += 2 * np.pi
        
        return [q1, q2, q3, d4, q5]


class TrajectoryPlannerNode(Node):
    """ROS 2 Node for planning and executing trajectories with analysis."""
    
    def __init__(self, x, y, z, duration=3.0, q5=0.0, output_dir='output'):
        super().__init__('trajectory_planner')
        
        self.target_position = (x, y, z)
        self.duration = duration
        self.q5 = q5
        self.output_dir = output_dir
        
        self.joint_names = [
            'base_rot1',
            'rot1_rot2', 
            'rot2_rot3',
            'rot3_prism1',
            'prism1_rot4'
        ]
        
        # Initialize components
        self.kinematics = BracoKinematics()
        self.planner = QuinticTrajectoryPlanner()
        self.dynamics = BracoDynamics()
        self.visualizer = TrajectoryVisualizer(output_dir)
        
        # Publisher
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/trajectory_command',
            10
        )
        
        # Timer
        self.timer = self.create_timer(0.5, self.execute_trajectory)
        self.executed = False
        
    def execute_trajectory(self):
        """Plan, analyze, visualize and execute the trajectory."""
        if self.executed:
            rclpy.shutdown()
            return
        
        x, y, z = self.target_position
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("PLANEJAMENTO DE TRAJETÓRIA - POLINÔMIO QUÍNTICO")
        self.get_logger().info("=" * 50)
        self.get_logger().info(f'Posição alvo: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        self.get_logger().info(f'Duração: {self.duration:.2f}s')
        
        # Calculate IK
        q_end = self.kinematics.inverse_kinematics(x, y, z, self.q5)
        q_start = [0.0, 0.0, 0.0, 0.0, 0.0]  # Home position
        
        self.get_logger().info("\nPosições das juntas calculadas:")
        for name, pos in zip(self.joint_names, q_end):
            if name == 'rot3_prism1':
                self.get_logger().info(f'  {name}: {pos*1000:.2f} mm')
            else:
                self.get_logger().info(f'  {name}: {np.degrees(pos):.2f}°')
        
        # Plan trajectory with quintic polynomial
        self.get_logger().info("\nPlanejando trajetória com polinômio quíntico...")
        trajectory_data = self.planner.plan_trajectory(q_start, q_end, self.duration, num_points=100)
        
        # Calculate torques along trajectory
        self.get_logger().info("Calculando torques dinâmicos...")
        num_points = len(trajectory_data['time'])
        torques = np.zeros((5, num_points))
        
        for i in range(num_points):
            q = trajectory_data['positions'][:, i]
            qd = trajectory_data['velocities'][:, i]
            qdd = trajectory_data['accelerations'][:, i]
            torques[:, i] = self.dynamics.compute_torques(q, qd, qdd)
        
        # Generate visualizations
        self.get_logger().info(f"\nGerando gráficos na pasta '{self.output_dir}/'...")
        self.visualizer.plot_positions(trajectory_data)
        self.visualizer.plot_velocities(trajectory_data)
        self.visualizer.plot_accelerations(trajectory_data)
        self.visualizer.plot_jerks(trajectory_data)
        self.visualizer.plot_torques(trajectory_data, torques)
        self.visualizer.plot_summary(trajectory_data, torques)
        self.visualizer.generate_report(trajectory_data, torques, q_start, q_end, 
                                        self.duration, self.target_position)
        
        # Create and publish ROS trajectory message
        self.get_logger().info("\nEnviando trajetória para o robô...")
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        
        # Send multiple points for smooth execution
        sample_indices = np.linspace(0, num_points-1, 20, dtype=int)
        
        for idx in sample_indices:
            point = JointTrajectoryPoint()
            point.positions = trajectory_data['positions'][:, idx].tolist()
            point.velocities = trajectory_data['velocities'][:, idx].tolist()
            point.accelerations = trajectory_data['accelerations'][:, idx].tolist()
            
            t = trajectory_data['time'][idx]
            point.time_from_start = Duration(
                sec=int(t),
                nanosec=int((t % 1) * 1e9)
            )
            msg.points.append(point)
        
        self.publisher.publish(msg)
        
        self.get_logger().info("\n" + "=" * 50)
        self.get_logger().info("TRAJETÓRIA ENVIADA COM SUCESSO!")
        self.get_logger().info(f"Gráficos salvos em: {os.path.abspath(self.output_dir)}/")
        self.get_logger().info("=" * 50)
        
        self.executed = True


def main():
    parser = argparse.ArgumentParser(
        description='Plan and execute quintic polynomial trajectory for Braco manipulator',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemplos:
    ros2 run braco_description trajectory_planner.py 0.2 0.1 0.15
    ros2 run braco_description trajectory_planner.py 0.3 0.2 0.2 --duration 5.0 --output plots
        """
    )
    parser.add_argument('x', type=float, help='Posição X alvo (metros)')
    parser.add_argument('y', type=float, help='Posição Y alvo (metros)')
    parser.add_argument('z', type=float, help='Posição Z alvo (metros)')
    parser.add_argument('--duration', '-d', type=float, default=3.0,
                        help='Duração da trajetória em segundos (padrão: 3.0)')
    parser.add_argument('--output', '-o', type=str, default='output',
                        help='Pasta de saída para os gráficos (padrão: output)')
    parser.add_argument('--rotation', '-r', type=float, default=0.0,
                        help='Rotação do end-effector em radianos (padrão: 0.0)')
    
    # Filter ROS arguments
    filtered_args = []
    for arg in sys.argv[1:]:
        if arg.startswith('--ros-args'):
            break
        filtered_args.append(arg)
    
    try:
        args = parser.parse_args(filtered_args)
    except SystemExit:
        print("\nUso: ros2 run braco_description trajectory_planner.py <x> <y> <z> [opções]")
        print("Exemplo: ros2 run braco_description trajectory_planner.py 0.2 0.1 0.15 --duration 3.0")
        return
    
    rclpy.init()
    node = TrajectoryPlannerNode(args.x, args.y, args.z, args.duration, args.rotation, args.output)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
