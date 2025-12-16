#!/usr/bin/env python3
"""
Interactive Trajectory Maker for ROS 2 - Cartesian End-Effector Control

Define trajectories by specifying end-effector positions (x, y, z).
Uses inverse kinematics to compute joint configurations.

Based on the RRRPR manipulator from peter_corke scripts.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import threading
import json
import os
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

# Robot DH Parameters (from peter_corke/scripts/rrrpr_plot.py)
L1 = 0.4   # Base height
L2 = 0.8   # Link 2 length
L3 = 0.7   # Link 3 length
L4 = 0.2   # Prismatic offset
L5 = 0.3   # End effector


class RRRPRKinematics:
    """Kinematics for the RRRPR subsea manipulator"""
    
    def __init__(self):
        self.l1, self.l2, self.l3, self.l4, self.l5 = L1, L2, L3, L4, L5
        
        # Joint limits
        self.q1_lim = (-np.pi, np.pi)
        self.q2_lim = (-np.pi/2, np.pi/2)
        self.q3_lim = (-2*np.pi/3, 2*np.pi/3)
        self.d4_lim = (0.0, 0.30)
        self.q5_lim = (-np.pi, np.pi)
        
    def forward_kinematics(self, q):
        """Compute end-effector position from joint angles q=[q1,q2,q3,d4,q5]"""
        q1, q2, q3, d4, q5 = q
        c1, s1 = np.cos(q1), np.sin(q1)
        c2, s2 = np.cos(q2), np.sin(q2)
        c23 = np.cos(q2 + q3)
        s23 = np.sin(q2 + q3)
        
        x = c1 * (self.l2 * c2 + self.l3 * c23)
        y = s1 * (self.l2 * c2 + self.l3 * c23)
        z = self.l1 + self.l2 * s2 + self.l3 * s23 - (self.l4 + d4)
        
        return np.array([x, y, z])
    
    def inverse_kinematics(self, target_pos, q_current=None):
        """Compute joint angles from end-effector position (x,y,z)"""
        x, y, z = target_pos
        
        # q1: base rotation
        q1 = np.arctan2(y, x)
        
        # Distance in xy plane
        r = np.sqrt(x**2 + y**2)
        
        # Try different d4 values to find reachable config
        for d4 in np.linspace(0.0, 0.3, 15):
            z_eff = z - self.l1 + self.l4 + d4
            
            # 2R planar arm IK
            D = (r**2 + z_eff**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
            
            if abs(D) <= 1.0:
                # q3 (elbow)
                q3 = np.arctan2(-np.sqrt(1 - D**2), D)
                
                # q2 (shoulder)
                k1 = self.l2 + self.l3 * np.cos(q3)
                k2 = self.l3 * np.sin(q3)
                q2 = np.arctan2(z_eff, r) - np.arctan2(k2, k1)
                
                q5 = 0.0
                q = np.array([q1, q2, q3, d4, q5])
                q = self.clip_to_limits(q)
                
                # Verify
                pos_check = self.forward_kinematics(q)
                if np.linalg.norm(pos_check - target_pos) < 0.05:
                    return q
        
        return None
    
    def clip_to_limits(self, q):
        limits = [self.q1_lim, self.q2_lim, self.q3_lim, self.d4_lim, self.q5_lim]
        return np.array([np.clip(q[i], limits[i][0], limits[i][1]) for i in range(5)])


class InteractiveTrajectoryMaker(Node):
    """Interactive trajectory maker with Cartesian end-effector control"""
    
    def __init__(self):
        super().__init__('interactive_trajectory_maker')
        
        self.kin = RRRPRKinematics()
        # Joint names matching your URDF exactly
        self.joint_names = ['base_rot1', 'rot1_rot2', 'rot2_rot3', 'rot3_prism1', 'prism1_rot4']
        self.current_q = np.zeros(5)
        self.cartesian_waypoints = []
        self.state_lock = threading.Lock()
        self.running = True
        self.rate_hz = 50
        self.timer_period = 1.0 / self.rate_hz
        
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        
        # Publisher for joint_states (for RViz visualization)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', qos)
        
        self.get_logger().info('Trajectory Maker initialized (Cartesian mode)')
        self.get_logger().info('Publishing to /joint_states for RViz visualization')
        
    def get_q(self):
        with self.state_lock:
            return self.current_q.copy()
    
    def get_pos(self):
        return self.kin.forward_kinematics(self.get_q())
    
    def publish_q(self, q):
        """Publish joint states for RViz visualization"""
        with self.state_lock:
            self.current_q = q.copy()
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = q.tolist()
        msg.velocity = [0.0] * 5
        msg.effort = [0.0] * 5
        self.joint_state_pub.publish(msg)
    
    def move_to_xyz(self, xyz, duration=3.0):
        """Move end-effector to Cartesian position"""
        target = np.array(xyz)
        q_target = self.kin.inverse_kinematics(target, self.get_q())
        
        if q_target is None:
            self.get_logger().error(f'Target ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) unreachable!')
            return False
        
        return self.move_to_q(q_target, duration)
    
    def move_to_q(self, q_target, duration=3.0):
        """Move to joint configuration"""
        q_target = self.kin.clip_to_limits(np.array(q_target))
        q_start = self.get_q()
        steps = int(duration * self.rate_hz)
        
        self.get_logger().info(f'Moving over {duration:.1f}s')
        
        for i in range(steps + 1):
            if not self.running:
                break
            t = i / float(steps)
            t_smooth = 3*t**2 - 2*t**3
            q = q_start + t_smooth * (q_target - q_start)
            self.publish_q(q)
            rclpy.spin_once(self, timeout_sec=self.timer_period)
        
        self.get_logger().info('Done')
        return True
    
    def add_waypoint(self, xyz=None):
        if xyz is None:
            pos = self.get_pos()
        else:
            pos = np.array(xyz)
            if self.kin.inverse_kinematics(pos, self.get_q()) is None:
                self.get_logger().error(f'Position unreachable!')
                return False
        
        self.cartesian_waypoints.append(pos.copy())
        self.get_logger().info(f'Waypoint {len(self.cartesian_waypoints)}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})')
        return True
    
    def remove_waypoint(self):
        if self.cartesian_waypoints:
            self.cartesian_waypoints.pop()
            self.get_logger().info('Removed last waypoint')
    
    def clear_waypoints(self):
        self.cartesian_waypoints = []
        self.get_logger().info('Cleared waypoints')
    
    def execute_trajectory(self, dur=2.0, pause=0.5):
        if not self.cartesian_waypoints:
            self.get_logger().error('No waypoints!')
            return
        
        self.get_logger().info(f'Executing {len(self.cartesian_waypoints)} waypoints')
        
        for i, wp in enumerate(self.cartesian_waypoints):
            if not self.running:
                break
            self.get_logger().info(f'[{i+1}/{len(self.cartesian_waypoints)}]')
            self.move_to_xyz(wp, dur)
            if pause > 0:
                import time
                time.sleep(pause)
        
        self.get_logger().info('Trajectory complete!')
    
    def go_home(self, dur=3.0):
        self.get_logger().info('Going home')
        self.move_to_q(np.zeros(5), dur)
    
    def save_traj(self, filename=None):
        if not self.cartesian_waypoints:
            print("No waypoints to save")
            return
        
        save_dir = os.path.expanduser('~/.ros2/trajectories')
        os.makedirs(save_dir, exist_ok=True)
        
        if filename is None:
            i = 1
            while os.path.exists(os.path.join(save_dir, f'traj_{i}.json')):
                i += 1
            filename = f'traj_{i}.json'
        
        filepath = os.path.join(save_dir, filename)
        data = {'waypoints': [wp.tolist() for wp in self.cartesian_waypoints]}
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        print(f'Saved to {filepath}')
    
    def load_traj(self, filename):
        save_dir = os.path.expanduser('~/.ros2/trajectories')
        filepath = os.path.join(save_dir, filename) if not os.path.isabs(filename) else filename
        
        if not os.path.exists(filepath):
            print(f'File not found: {filepath}')
            return
        
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        self.cartesian_waypoints = [np.array(wp) for wp in data['waypoints']]
        print(f'Loaded {len(self.cartesian_waypoints)} waypoints')
    
    def list_traj(self):
        save_dir = os.path.expanduser('~/.ros2/trajectories')
        if os.path.exists(save_dir):
            files = [f for f in os.listdir(save_dir) if f.endswith('.json')]
            return files
        return []
    
    def status(self):
        q = self.get_q()
        pos = self.kin.forward_kinematics(q)
        
        print("\n" + "="*50)
        print("STATUS")
        print("="*50)
        print(f"End-Effector: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
        print(f"Joints: q1={np.rad2deg(q[0]):.1f}°, q2={np.rad2deg(q[1]):.1f}°, q3={np.rad2deg(q[2]):.1f}°, d4={q[3]:.3f}m, q5={np.rad2deg(q[4]):.1f}°")
        print(f"Waypoints: {len(self.cartesian_waypoints)}")
        for i, wp in enumerate(self.cartesian_waypoints):
            print(f"  [{i+1}] ({wp[0]:.3f}, {wp[1]:.3f}, {wp[2]:.3f})")
        print("="*50 + "\n")
    
    def help(self):
        print("""
============================================================
TRAJECTORY MAKER - CARTESIAN CONTROL
============================================================

Commands:
  goto x y z [dur]    - Move to position (meters)
  add [x y z]         - Add waypoint (current if no args)
  remove              - Remove last waypoint
  clear               - Clear all waypoints
  exec [dur]          - Execute trajectory
  home                - Go to home position
  status              - Show current state
  save [file]         - Save trajectory
  load file           - Load trajectory
  list                - List saved trajectories
  fk q1 q2 q3 d4 q5   - Forward kinematics test
  help                - Show this help
  quit                - Exit

Example:
  >>> goto 0.8 0.3 0.5
  >>> add
  >>> goto 0.6 -0.4 0.3
  >>> add
  >>> exec 2.0
============================================================
""")
    
    def run(self):
        self.help()
        
        while self.running and rclpy.ok():
            try:
                line = input(">>> ").strip()
                if not line:
                    continue
                
                parts = line.split()
                cmd = parts[0].lower()
                args = parts[1:]
                
                if cmd in ['quit', 'exit', 'q']:
                    break
                elif cmd == 'help':
                    self.help()
                elif cmd == 'status':
                    self.status()
                elif cmd == 'goto':
                    if len(args) < 3:
                        print("Usage: goto x y z [duration]")
                    else:
                        x, y, z = float(args[0]), float(args[1]), float(args[2])
                        dur = float(args[3]) if len(args) > 3 else 3.0
                        self.move_to_xyz([x, y, z], dur)
                elif cmd == 'add':
                    if len(args) >= 3:
                        x, y, z = float(args[0]), float(args[1]), float(args[2])
                        self.add_waypoint([x, y, z])
                    else:
                        self.add_waypoint()
                elif cmd == 'remove':
                    self.remove_waypoint()
                elif cmd == 'clear':
                    self.clear_waypoints()
                elif cmd == 'exec':
                    dur = float(args[0]) if args else 2.0
                    self.execute_trajectory(dur)
                elif cmd == 'home':
                    dur = float(args[0]) if args else 3.0
                    self.go_home(dur)
                elif cmd == 'save':
                    self.save_traj(args[0] if args else None)
                elif cmd == 'load':
                    if args:
                        self.load_traj(args[0])
                    else:
                        print("Usage: load filename")
                elif cmd == 'list':
                    files = self.list_traj()
                    print("Saved:", files if files else "None")
                elif cmd == 'fk':
                    if len(args) >= 5:
                        q = [float(x) for x in args[:5]]
                        pos = self.kin.forward_kinematics(q)
                        print(f"Position: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
                    else:
                        print("Usage: fk q1 q2 q3 d4 q5")
                else:
                    print(f"Unknown: {cmd}. Type 'help'")
                    
            except ValueError as e:
                print(f"Error: {e}")
            except KeyboardInterrupt:
                break
            except EOFError:
                break
        
        self.running = False
        print("\nBye!")


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveTrajectoryMaker()
    
    try:
        import time
        time.sleep(0.5)
        node.run()
    finally:
        node.running = False
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
