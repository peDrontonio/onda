#!/usr/bin/env python3
"""
Interactive Trajectory Maker for RRRPR Manipulator

Allows user to:
1. Define waypoints interactively (joint space or Cartesian)
2. Visualize the robot at each waypoint
3. Execute trajectories between waypoints
4. Save/load trajectory sequences

Author: Trajectory Maker
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Button, TextBox
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH, jtraj
from spatialmath import SE3
import json
import os

# ============================================
# 1) ROBOT DEFINITION (same as rrrpr_plot.py)
# ============================================

l1, l2, l3, l4, l5 = 0.4, 0.8, 0.7, 0.2, 0.3
d4_lim = (0.0, 0.30)

q1_lim = (-np.pi, np.pi)
q2_lim = (-np.pi/2, np.pi/2)
q3_lim = (-2*np.pi/3, 2*np.pi/3)
q5_lim = (-np.pi, np.pi)

arm = DHRobot([
    RevoluteDH(a=0.0, d=l1, alpha=np.pi/2, qlim=q1_lim),
    RevoluteDH(a=l2, d=0.0, alpha=0.0, qlim=q2_lim),
    RevoluteDH(a=l3, d=0.0, alpha=0.0, qlim=q3_lim),
    PrismaticDH(a=0.0, theta=0.0, alpha=-np.pi/2, offset=l4, qlim=d4_lim),
    RevoluteDH(a=0.0, d=l5, alpha=np.pi/2, qlim=q5_lim),
], name="RRRPR")


class TrajectoryMaker:
    """Interactive trajectory maker for the RRRPR manipulator"""
    
    def __init__(self):
        self.robot = arm
        self.waypoints = []  # List of joint configurations
        self.trajectory = None  # Full trajectory
        self.current_q = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.trajectory_points = []  # End-effector path for visualization
        
        # Setup figure
        self.setup_figure()
        
    def setup_figure(self):
        """Setup the matplotlib figure with controls"""
        self.fig = plt.figure(figsize=(14, 10))
        
        # 3D plot for robot visualization
        self.ax3d = self.fig.add_axes([0.05, 0.35, 0.55, 0.60], projection='3d')
        self.ax3d.set_xlim([-2, 2])
        self.ax3d.set_ylim([-2, 2])
        self.ax3d.set_zlim([0, 2.5])
        self.ax3d.set_xlabel('X (m)')
        self.ax3d.set_ylabel('Y (m)')
        self.ax3d.set_zlabel('Z (m)')
        self.ax3d.set_title('RRRPR Manipulator - Trajectory Maker')
        
        # Info panel
        self.ax_info = self.fig.add_axes([0.65, 0.35, 0.32, 0.60])
        self.ax_info.axis('off')
        self.info_text = self.ax_info.text(
            0.0, 1.0, "", va="top", ha="left", fontsize=9,
            transform=self.ax_info.transAxes, family='monospace'
        )
        
        # Input boxes for joint values
        self.setup_input_boxes()
        
        # Buttons
        self.setup_buttons()
        
        # Initialize robot plot
        self.line_robot, = self.ax3d.plot([], [], [], 'o-', linewidth=3, color='tab:red', markersize=8)
        self.traj_line, = self.ax3d.plot([], [], [], '-', linewidth=1.5, color='tab:blue', alpha=0.6)
        self.waypoint_markers, = self.ax3d.plot([], [], [], 'g*', markersize=15)
        
        # Update initial display
        self.update_robot_display()
        self.update_info_display()
        
    def setup_input_boxes(self):
        """Setup text input boxes for joint values"""
        box_height = 0.04
        box_width = 0.08
        start_y = 0.25
        
        # Labels and boxes for each joint
        self.joint_boxes = []
        joint_labels = ['q1 (rad):', 'q2 (rad):', 'q3 (rad):', 'd4 (m):', 'q5 (rad):']
        initial_vals = ['0.0', '0.0', '0.0', '0.0', '0.0']
        
        for i, (label, init_val) in enumerate(zip(joint_labels, initial_vals)):
            y_pos = start_y - i * 0.05
            
            # Label
            self.fig.text(0.05, y_pos + 0.02, label, fontsize=10)
            
            # Text box
            ax_box = self.fig.add_axes([0.15, y_pos, box_width, box_height])
            text_box = TextBox(ax_box, '', initial=init_val)
            text_box.on_submit(lambda val, idx=i: self.on_joint_change(val, idx))
            self.joint_boxes.append(text_box)
            
    def setup_buttons(self):
        """Setup control buttons"""
        button_width = 0.12
        button_height = 0.05
        
        # Add Waypoint button
        ax_add = self.fig.add_axes([0.30, 0.20, button_width, button_height])
        self.btn_add = Button(ax_add, 'Add Waypoint')
        self.btn_add.on_clicked(self.add_waypoint)
        
        # Remove Last Waypoint button
        ax_remove = self.fig.add_axes([0.30, 0.14, button_width, button_height])
        self.btn_remove = Button(ax_remove, 'Remove Last')
        self.btn_remove.on_clicked(self.remove_last_waypoint)
        
        # Clear All button
        ax_clear = self.fig.add_axes([0.30, 0.08, button_width, button_height])
        self.btn_clear = Button(ax_clear, 'Clear All')
        self.btn_clear.on_clicked(self.clear_waypoints)
        
        # Execute Trajectory button
        ax_exec = self.fig.add_axes([0.45, 0.20, button_width, button_height])
        self.btn_exec = Button(ax_exec, 'Execute Traj')
        self.btn_exec.on_clicked(self.execute_trajectory)
        
        # Go to Current button
        ax_goto = self.fig.add_axes([0.45, 0.14, button_width, button_height])
        self.btn_goto = Button(ax_goto, 'Go to Input')
        self.btn_goto.on_clicked(self.go_to_input)
        
        # Save Trajectory button
        ax_save = self.fig.add_axes([0.60, 0.20, button_width, button_height])
        self.btn_save = Button(ax_save, 'Save Traj')
        self.btn_save.on_clicked(self.save_trajectory)
        
        # Load Trajectory button
        ax_load = self.fig.add_axes([0.60, 0.14, button_width, button_height])
        self.btn_load = Button(ax_load, 'Load Traj')
        self.btn_load.on_clicked(self.load_trajectory)
        
        # Home button
        ax_home = self.fig.add_axes([0.60, 0.08, button_width, button_height])
        self.btn_home = Button(ax_home, 'Go Home')
        self.btn_home.on_clicked(self.go_home)
        
    def on_joint_change(self, val, joint_idx):
        """Handle joint value change from text box"""
        try:
            new_val = float(val)
            self.current_q[joint_idx] = new_val
            self.update_robot_display()
            self.update_info_display()
        except ValueError:
            print(f"Invalid value for joint {joint_idx + 1}: {val}")
            
    def get_input_values(self):
        """Get current values from input boxes"""
        values = []
        for box in self.joint_boxes:
            try:
                values.append(float(box.text))
            except ValueError:
                values.append(0.0)
        return np.array(values)
    
    def set_input_values(self, q):
        """Set values in input boxes"""
        for i, box in enumerate(self.joint_boxes):
            box.set_val(f"{q[i]:.4f}")
            
    def forward_chain_points(self, q):
        """Get points along the kinematic chain for visualization"""
        points = [np.array([0.0, 0.0, 0.0])]
        T = np.eye(4)
        
        for i in range(self.robot.n):
            T = T @ self.robot.links[i].A(q[i]).A
            p = T[0:3, 3]
            points.append(p)
            
        pts = np.array(points)
        return pts[:, 0], pts[:, 1], pts[:, 2]
    
    def update_robot_display(self):
        """Update the robot visualization"""
        x, y, z = self.forward_chain_points(self.current_q)
        self.line_robot.set_data(x, y)
        self.line_robot.set_3d_properties(z)
        
        # Update waypoint markers
        if self.waypoints:
            wx, wy, wz = [], [], []
            for wp in self.waypoints:
                T = self.robot.fkine(wp)
                pos = T.t
                wx.append(pos[0])
                wy.append(pos[1])
                wz.append(pos[2])
            self.waypoint_markers.set_data(wx, wy)
            self.waypoint_markers.set_3d_properties(wz)
        else:
            self.waypoint_markers.set_data([], [])
            self.waypoint_markers.set_3d_properties([])
            
        self.fig.canvas.draw_idle()
        
    def update_info_display(self):
        """Update the information panel"""
        T = self.robot.fkine(self.current_q)
        pos = T.t
        rpy = T.rpy(order="zyx", unit="deg")
        
        info_str = (
            "═══ CURRENT CONFIGURATION ═══\n\n"
            f"  q1 = {self.current_q[0]:+.4f} rad ({np.rad2deg(self.current_q[0]):+.2f}°)\n"
            f"  q2 = {self.current_q[1]:+.4f} rad ({np.rad2deg(self.current_q[1]):+.2f}°)\n"
            f"  q3 = {self.current_q[2]:+.4f} rad ({np.rad2deg(self.current_q[2]):+.2f}°)\n"
            f"  d4 = {self.current_q[3]:+.4f} m\n"
            f"  q5 = {self.current_q[4]:+.4f} rad ({np.rad2deg(self.current_q[4]):+.2f}°)\n\n"
            "═══ END-EFFECTOR POSE ═══\n\n"
            f"  Position:\n"
            f"    x = {pos[0]:+.4f} m\n"
            f"    y = {pos[1]:+.4f} m\n"
            f"    z = {pos[2]:+.4f} m\n\n"
            f"  Orientation (RPY):\n"
            f"    yaw   = {rpy[0]:+.2f}°\n"
            f"    pitch = {rpy[1]:+.2f}°\n"
            f"    roll  = {rpy[2]:+.2f}°\n\n"
            "═══ WAYPOINTS ═══\n\n"
            f"  Total waypoints: {len(self.waypoints)}\n"
        )
        
        # Add waypoint list
        for i, wp in enumerate(self.waypoints[-5:]):  # Show last 5
            T_wp = self.robot.fkine(wp)
            pos_wp = T_wp.t
            info_str += f"  [{i+1}] pos=({pos_wp[0]:.2f}, {pos_wp[1]:.2f}, {pos_wp[2]:.2f})\n"
            
        if len(self.waypoints) > 5:
            info_str += f"  ... and {len(self.waypoints) - 5} more\n"
            
        self.info_text.set_text(info_str)
        
    def add_waypoint(self, event=None):
        """Add current configuration as a waypoint"""
        q = self.get_input_values()
        
        # Validate joint limits
        limits = [q1_lim, q2_lim, q3_lim, d4_lim, q5_lim]
        for i, (val, lim) in enumerate(zip(q, limits)):
            if not (lim[0] <= val <= lim[1]):
                print(f"Warning: Joint {i+1} value {val:.4f} outside limits {lim}")
                q[i] = np.clip(val, lim[0], lim[1])
                
        self.waypoints.append(q.copy())
        self.current_q = q.copy()
        print(f"Added waypoint {len(self.waypoints)}: {q}")
        
        self.update_robot_display()
        self.update_info_display()
        
    def remove_last_waypoint(self, event=None):
        """Remove the last waypoint"""
        if self.waypoints:
            removed = self.waypoints.pop()
            print(f"Removed waypoint: {removed}")
            self.update_robot_display()
            self.update_info_display()
        else:
            print("No waypoints to remove")
            
    def clear_waypoints(self, event=None):
        """Clear all waypoints"""
        self.waypoints = []
        self.trajectory_points = []
        self.traj_line.set_data([], [])
        self.traj_line.set_3d_properties([])
        print("Cleared all waypoints")
        self.update_robot_display()
        self.update_info_display()
        
    def go_to_input(self, event=None):
        """Move robot display to input values"""
        self.current_q = self.get_input_values()
        self.update_robot_display()
        self.update_info_display()
        
    def go_home(self, event=None):
        """Move robot to home position"""
        self.current_q = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.set_input_values(self.current_q)
        self.update_robot_display()
        self.update_info_display()
        
    def execute_trajectory(self, event=None):
        """Execute trajectory through all waypoints"""
        if len(self.waypoints) < 2:
            print("Need at least 2 waypoints to execute trajectory")
            return
            
        print(f"Executing trajectory through {len(self.waypoints)} waypoints...")
        
        # Generate full trajectory
        steps_per_segment = 50
        full_traj = []
        self.trajectory_points = []
        
        for i in range(len(self.waypoints) - 1):
            traj_segment = jtraj(self.waypoints[i], self.waypoints[i + 1], steps_per_segment)
            full_traj.append(traj_segment.q)
            
        full_traj = np.vstack(full_traj)
        
        # Animate
        for q in full_traj:
            self.current_q = q
            self.set_input_values(q)
            
            # Store end-effector position for trail
            T = self.robot.fkine(q)
            self.trajectory_points.append(T.t.copy())
            
            # Update trajectory line
            if len(self.trajectory_points) > 1:
                pts = np.array(self.trajectory_points)
                self.traj_line.set_data(pts[:, 0], pts[:, 1])
                self.traj_line.set_3d_properties(pts[:, 2])
                
            self.update_robot_display()
            plt.pause(0.02)
            
        print("Trajectory execution complete!")
        self.update_info_display()
        
    def save_trajectory(self, event=None):
        """Save waypoints to file"""
        if not self.waypoints:
            print("No waypoints to save")
            return
            
        save_path = os.path.join(os.path.dirname(__file__), 'saved_trajectories')
        os.makedirs(save_path, exist_ok=True)
        
        # Find next available filename
        i = 1
        while os.path.exists(os.path.join(save_path, f'trajectory_{i}.json')):
            i += 1
            
        filename = os.path.join(save_path, f'trajectory_{i}.json')
        
        data = {
            'waypoints': [wp.tolist() for wp in self.waypoints],
            'robot_params': {
                'l1': l1, 'l2': l2, 'l3': l3, 'l4': l4, 'l5': l5
            }
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
            
        print(f"Saved {len(self.waypoints)} waypoints to {filename}")
        
    def load_trajectory(self, event=None):
        """Load waypoints from file"""
        save_path = os.path.join(os.path.dirname(__file__), 'saved_trajectories')
        
        if not os.path.exists(save_path):
            print("No saved trajectories found")
            return
            
        # Find most recent file
        files = sorted([f for f in os.listdir(save_path) if f.endswith('.json')])
        if not files:
            print("No trajectory files found")
            return
            
        filename = os.path.join(save_path, files[-1])
        
        with open(filename, 'r') as f:
            data = json.load(f)
            
        self.waypoints = [np.array(wp) for wp in data['waypoints']]
        
        if self.waypoints:
            self.current_q = self.waypoints[-1].copy()
            self.set_input_values(self.current_q)
            
        print(f"Loaded {len(self.waypoints)} waypoints from {filename}")
        self.update_robot_display()
        self.update_info_display()
        
    def run(self):
        """Run the trajectory maker"""
        print("\n" + "="*60)
        print("  RRRPR MANIPULATOR - INTERACTIVE TRAJECTORY MAKER")
        print("="*60)
        print("\nInstructions:")
        print("  1. Enter joint values in the text boxes")
        print("  2. Click 'Go to Input' to move robot to those values")
        print("  3. Click 'Add Waypoint' to save current position")
        print("  4. Repeat for all desired waypoints")
        print("  5. Click 'Execute Traj' to animate through waypoints")
        print("  6. Use 'Save Traj' and 'Load Traj' to store/recall")
        print("\nJoint Limits:")
        print(f"  q1: [{np.rad2deg(q1_lim[0]):.1f}°, {np.rad2deg(q1_lim[1]):.1f}°]")
        print(f"  q2: [{np.rad2deg(q2_lim[0]):.1f}°, {np.rad2deg(q2_lim[1]):.1f}°]")
        print(f"  q3: [{np.rad2deg(q3_lim[0]):.1f}°, {np.rad2deg(q3_lim[1]):.1f}°]")
        print(f"  d4: [{d4_lim[0]:.2f}m, {d4_lim[1]:.2f}m]")
        print(f"  q5: [{np.rad2deg(q5_lim[0]):.1f}°, {np.rad2deg(q5_lim[1]):.1f}°]")
        print("="*60 + "\n")
        
        plt.show()


def main():
    maker = TrajectoryMaker()
    maker.run()


if __name__ == '__main__':
    main()