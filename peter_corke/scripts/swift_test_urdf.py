#!/usr/bin/env python3
"""
Swift visualization for the RRRPR manipulator using URDF model
"""
import numpy as np
import tkinter as tk
from tkinter import ttk
import roboticstoolbox as rtb
import swift
import os

# ============================================
# 1) LOAD URDF MODEL
# ============================================

# Set up paths for the URDF package
package_path = "/home/host/onda/ros_ws/braco_description"
urdf_file = os.path.join(package_path, "urdf", "braco.urdf")

print(f"Loading URDF from: {urdf_file}")

# Configure roboticstoolbox to find package:// URIs
# We need to tell it where braco_description package is located
import roboticstoolbox.tools.urdf as urdf_tools

# Create a simple package resolver
def package_resolver(package_name):
    """Resolve package:// URIs to actual file paths"""
    if package_name == "braco_description":
        return package_path
    return None

# Monkey-patch or configure the URDF loader
# RTB uses its own resolver, so we'll handle this by loading with full paths

# Load the URDF model
try:
    # Read URDF and replace package:// with absolute paths for loading
    with open(urdf_file, 'r') as f:
        urdf_string = f.read()
    
    # Replace package:// URIs with absolute paths for roboticstoolbox
    urdf_string = urdf_string.replace(
        'package://braco_description',
        f'file://{package_path}'
    )
    
    # Save temporary URDF with resolved paths
    temp_urdf = "/tmp/braco_resolved.urdf"
    with open(temp_urdf, 'w') as f:
        f.write(urdf_string)
    
    # Load the robot model
    arm = rtb.Robot.URDF(temp_urdf)
    print(f"✓ Successfully loaded robot: {arm.name}")
    print(f"✓ Number of joints: {arm.n}")
    print(f"✓ Joint names: {[link.name for link in arm.links if link.isjoint]}")
    
except Exception as e:
    print(f"✗ Error loading URDF: {e}")
    import traceback
    traceback.print_exc()
    exit(1)

# Get joint limits
qlims = []
for link in arm.links:
    if link.isjoint:
        if link.qlim is not None:
            qlims.append(link.qlim)
        else:
            # Default limits if not specified
            if link.isprismatic:
                qlims.append([0.0, 0.3])
            else:
                qlims.append([-np.pi, np.pi])

print(f"Joint limits: {qlims}")

# Initial pose
q_current = np.zeros(arm.n)
arm.q = q_current.copy()

# ============================================
# 2) INICIA O SWIFT (3D)
# ============================================

env = swift.Swift()
env.launch(realtime=False)
env.add(arm)

print("✓ Swift environment initialized")

# ============================================
# 3) INTERFACE TKINTER COM SLIDERS
# ============================================

import os
if os.environ.get('DISPLAY', '') == '':
    print('No display found. Running in headless mode.')
    print('Swift simulation is running. Open the browser manually if needed.')
    # Keep the script running
    try:
        while True:
            env.step(0.05)
    except KeyboardInterrupt:
        print("Stopping...")
    import sys
    sys.exit(0)

root = tk.Tk()
root.title("Controle do braço RRRPR - URDF Model")

mainframe = ttk.Frame(root, padding="10 10 10 10")
mainframe.grid(row=0, column=0, sticky="nsew")

root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

# Create slider variables dynamically based on number of joints
slider_vars = []
for i in range(arm.n):
    slider_vars.append(tk.DoubleVar(value=0.0))

# Texto do Jacobiano
jacobian_text = tk.StringVar()
jacobian_text.set("Jacobiano atual:\n")

def get_q_from_vars():
    return np.array([var.get() for var in slider_vars])

def update_robot_from_sliders(*args):
    """Atualiza o robô no Swift e o Jacobiano quando os sliders mudam."""
    global q_current
    q_current = get_q_from_vars()

    # Atualiza junta e simulação
    arm.q = q_current
    env.step(0.02)

    # Jacobiano na base
    try:
        J = arm.jacob0(q_current)
        jacobian_text.set(f"Jacobiano atual ({J.shape[0]}x{J.shape[1]}):\n{np.round(J, 3)}")
    except Exception as e:
        jacobian_text.set(f"Jacobiano: Error - {e}")

# Create sliders for each joint
joint_names = ["q1 (base)", "q2 (shoulder)", "q3 (elbow)", "d4 (prism)", "q5 (wrist)"]
for i in range(arm.n):
    joint_name = joint_names[i] if i < len(joint_names) else f"Joint {i+1}"
    
    ttk.Label(mainframe, text=joint_name).grid(column=0, row=i, sticky="w")
    
    slider = ttk.Scale(
        mainframe, 
        from_=qlims[i][0], 
        to=qlims[i][1],
        orient='horizontal', 
        variable=slider_vars[i], 
        command=update_robot_from_sliders
    )
    slider.grid(column=1, row=i, sticky="ew")

# Label do Jacobiano
jacobian_label = ttk.Label(mainframe, textvariable=jacobian_text, justify="left")
jacobian_label.grid(column=0, row=arm.n, columnspan=2, sticky="w", pady=(10, 10))

# ============================================
# 4) ANIMAR TRAJETÓRIA (BOTÃO)
# ============================================

def animate_trajectory():
    """Gera uma trajetória de q_current até um alvo e anima no Swift + sliders."""
    global q_current

    # Define target based on joint limits
    q_target = np.array([
        0.4,              # gira base
        -0.4,             # ombro baixa
        0.25,             # cotovelo dobra
        0.05,             # estende prismática (dentro do limite 0-0.1)
        np.pi/4           # gira wrist
    ])
    
    # Ensure target is within limits
    for i in range(len(q_target)):
        q_target[i] = np.clip(q_target[i], qlims[i][0], qlims[i][1])

    steps = 80
    traj = rtb.tools.trajectory.jtraj(q_current, q_target, steps)

    def step_traj(i=0):
        global q_current
        if i >= steps:
            return

        q_current = traj.q[i, :]
        arm.q = q_current

        # atualiza sliders
        for j, var in enumerate(slider_vars):
            var.set(q_current[j])

        # atualiza simulação e jacobiano
        env.step(0.02)
        try:
            J = arm.jacob0(q_current)
            jacobian_text.set(f"Jacobiano atual ({J.shape[0]}x{J.shape[1]}):\n{np.round(J, 3)}")
        except Exception as e:
            jacobian_text.set(f"Jacobiano: Error - {e}")

        # agenda próximo frame
        root.after(30, lambda: step_traj(i+1))

    step_traj(0)

anim_button = ttk.Button(mainframe, text="Animar trajetória", command=animate_trajectory)
anim_button.grid(column=0, row=arm.n+1, columnspan=2, pady=(10, 0))

# Configura expansão da UI
for col in range(2):
    mainframe.columnconfigure(col, weight=1)

print("✓ GUI initialized. Starting main loop...")
root.mainloop()
