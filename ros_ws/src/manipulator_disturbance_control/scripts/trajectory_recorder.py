#!/usr/bin/env python3
"""
Trajectory Recorder for ROS 2

Works with joint_state_publisher_gui:
1. Use the GUI sliders to position the robot
2. Press 'add' to save the current position as a waypoint
3. Execute the trajectory to replay all waypoints

Run alongside: ros2 launch braco_description display.launch.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import json
import os
import time
from sensor_msgs.msg import JointState


class TrajectoryRecorder(Node):
    """Record and playback trajectories using joint_state_publisher_gui"""
    
    def __init__(self):
        super().__init__('trajectory_recorder')
        
        self.current_positions = {}
        self.joint_names = []
        self.waypoints = []
        self.running = True
        self.rate_hz = 50
        
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        
        # Subscribe to joint states from joint_state_publisher_gui
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos
        )
        
        # Publisher to override joint states during playback
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', qos)
        
        self.get_logger().info('Trajectory Recorder started')
        self.get_logger().info('Use joint_state_publisher_gui to position the robot')
        self.get_logger().info('Then use commands here to record waypoints')
        
    def joint_state_callback(self, msg: JointState):
        """Capture current joint positions from GUI"""
        if not self.joint_names:
            self.joint_names = list(msg.name)
            self.get_logger().info(f'Detected joints: {self.joint_names}')
        
        for i, name in enumerate(msg.name):
            self.current_positions[name] = msg.position[i]
    
    def get_current_as_list(self):
        """Get current positions as ordered list"""
        if not self.joint_names:
            return None
        return [self.current_positions.get(name, 0.0) for name in self.joint_names]
    
    def add_waypoint(self):
        """Record current position as waypoint"""
        pos = self.get_current_as_list()
        if pos is None:
            self.get_logger().error('No joint states received yet!')
            return False
        
        self.waypoints.append(pos.copy())
        self.get_logger().info(f'Waypoint {len(self.waypoints)} added')
        
        # Print joint values
        print(f"  Joints: ", end="")
        for i, name in enumerate(self.joint_names):
            print(f"{name}={pos[i]:.3f}", end=" ")
        print()
        return True
    
    def remove_waypoint(self):
        """Remove last waypoint"""
        if self.waypoints:
            self.waypoints.pop()
            self.get_logger().info(f'Removed waypoint. {len(self.waypoints)} remaining')
        else:
            self.get_logger().warn('No waypoints to remove')
    
    def clear_waypoints(self):
        """Clear all waypoints"""
        self.waypoints = []
        self.get_logger().info('All waypoints cleared')
    
    def execute_trajectory(self, duration_per_segment=2.0, pause=0.5):
        """Play back the recorded trajectory"""
        if len(self.waypoints) < 1:
            self.get_logger().error('No waypoints recorded!')
            return
        
        self.get_logger().info(f'Executing {len(self.waypoints)} waypoints...')
        self.get_logger().warn('NOTE: Close joint_state_publisher_gui window to see playback!')
        
        # Get starting position
        start_pos = self.get_current_as_list()
        if start_pos is None:
            start_pos = self.waypoints[0]
        
        # Move through each waypoint
        all_waypoints = [start_pos] + self.waypoints
        
        for i in range(len(all_waypoints) - 1):
            start = np.array(all_waypoints[i])
            end = np.array(all_waypoints[i + 1])
            
            self.get_logger().info(f'Moving to waypoint {i + 1}/{len(self.waypoints)}')
            
            # Interpolate
            steps = int(duration_per_segment * self.rate_hz)
            for step in range(steps + 1):
                if not self.running:
                    return
                
                t = step / float(steps)
                t_smooth = 3*t**2 - 2*t**3  # Smoothstep
                pos = start + t_smooth * (end - start)
                
                # Publish joint state
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = self.joint_names
                msg.position = pos.tolist()
                self.joint_state_pub.publish(msg)
                
                time.sleep(1.0 / self.rate_hz)
            
            # Pause at waypoint
            if pause > 0:
                time.sleep(pause)
        
        self.get_logger().info('Trajectory complete!')
    
    def go_to_waypoint(self, index, duration=2.0):
        """Go to a specific waypoint"""
        if index < 1 or index > len(self.waypoints):
            self.get_logger().error(f'Invalid waypoint index. Have {len(self.waypoints)} waypoints')
            return
        
        target = np.array(self.waypoints[index - 1])
        start = np.array(self.get_current_as_list())
        
        self.get_logger().info(f'Going to waypoint {index}')
        
        steps = int(duration * self.rate_hz)
        for step in range(steps + 1):
            t = step / float(steps)
            t_smooth = 3*t**2 - 2*t**3
            pos = start + t_smooth * (target - start)
            
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = pos.tolist()
            self.joint_state_pub.publish(msg)
            
            time.sleep(1.0 / self.rate_hz)
    
    def save_trajectory(self, filename=None):
        """Save waypoints to file"""
        if not self.waypoints:
            print("No waypoints to save")
            return
        
        save_dir = os.path.expanduser('~/.ros2/trajectories')
        os.makedirs(save_dir, exist_ok=True)
        
        if filename is None:
            i = 1
            while os.path.exists(os.path.join(save_dir, f'traj_{i}.json')):
                i += 1
            filename = f'traj_{i}.json'
        
        if not filename.endswith('.json'):
            filename += '.json'
        
        filepath = os.path.join(save_dir, filename)
        data = {
            'joint_names': self.joint_names,
            'waypoints': self.waypoints
        }
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        print(f'Saved {len(self.waypoints)} waypoints to {filepath}')
    
    def load_trajectory(self, filename):
        """Load waypoints from file"""
        save_dir = os.path.expanduser('~/.ros2/trajectories')
        
        if not filename.endswith('.json'):
            filename += '.json'
        
        filepath = os.path.join(save_dir, filename) if not os.path.isabs(filename) else filename
        
        if not os.path.exists(filepath):
            print(f'File not found: {filepath}')
            return
        
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        self.waypoints = data['waypoints']
        loaded_joints = data.get('joint_names', [])
        print(f'Loaded {len(self.waypoints)} waypoints')
        print(f'Joints: {loaded_joints}')
    
    def list_trajectories(self):
        """List saved trajectories"""
        save_dir = os.path.expanduser('~/.ros2/trajectories')
        if os.path.exists(save_dir):
            files = [f for f in os.listdir(save_dir) if f.endswith('.json')]
            return files
        return []
    
    def status(self):
        """Print current status"""
        print("\n" + "="*60)
        print("TRAJECTORY RECORDER STATUS")
        print("="*60)
        
        pos = self.get_current_as_list()
        if pos:
            print("\nCurrent joint positions:")
            for i, name in enumerate(self.joint_names):
                print(f"  {name}: {pos[i]:.4f}")
        else:
            print("\nNo joint states received yet")
        
        print(f"\nRecorded waypoints: {len(self.waypoints)}")
        for i, wp in enumerate(self.waypoints):
            print(f"  [{i+1}] ", end="")
            for j, val in enumerate(wp):
                print(f"{val:.3f}", end=" ")
            print()
        
        print("="*60 + "\n")
    
    def help(self):
        """Print help"""
        print("""
============================================================
TRAJECTORY RECORDER
============================================================

Use joint_state_publisher_gui to position the robot,
then use these commands to record and play trajectories.

Commands:
  add           - Record current position as waypoint
  remove        - Remove last waypoint
  clear         - Clear all waypoints
  play [dur]    - Play trajectory (dur=seconds per segment)
  goto N [dur]  - Go to waypoint N
  status        - Show current state and waypoints
  save [name]   - Save trajectory to file
  load name     - Load trajectory from file
  list          - List saved trajectories
  loop N [dur]  - Play trajectory N times
  help          - Show this help
  quit          - Exit

Workflow:
  1. Position robot using GUI sliders
  2. Type 'add' to record position
  3. Repeat for all waypoints
  4. Close GUI window, then 'play' to see trajectory
  5. Use 'save' to keep for later

============================================================
""")
    
    def run(self):
        """Main interactive loop"""
        print("\nWaiting for joint states...")
        
        # Wait for first joint state message
        timeout = 10.0
        start = time.time()
        while not self.joint_names and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.joint_names:
            self.get_logger().error('No joint states received! Is joint_state_publisher running?')
            return
        
        self.help()
        
        while self.running and rclpy.ok():
            try:
                # Spin to get updates
                rclpy.spin_once(self, timeout_sec=0.01)
                
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
                elif cmd == 'add':
                    self.add_waypoint()
                elif cmd == 'remove':
                    self.remove_waypoint()
                elif cmd == 'clear':
                    self.clear_waypoints()
                elif cmd == 'play':
                    dur = float(args[0]) if args else 2.0
                    self.execute_trajectory(dur)
                elif cmd == 'goto':
                    if not args:
                        print("Usage: goto N [duration]")
                    else:
                        idx = int(args[0])
                        dur = float(args[1]) if len(args) > 1 else 2.0
                        self.go_to_waypoint(idx, dur)
                elif cmd == 'loop':
                    if not args:
                        print("Usage: loop N [duration]")
                    else:
                        n = int(args[0])
                        dur = float(args[1]) if len(args) > 1 else 2.0
                        for i in range(n):
                            print(f"Loop {i+1}/{n}")
                            self.execute_trajectory(dur)
                elif cmd == 'save':
                    self.save_trajectory(args[0] if args else None)
                elif cmd == 'load':
                    if args:
                        self.load_trajectory(args[0])
                    else:
                        print("Usage: load filename")
                elif cmd == 'list':
                    files = self.list_trajectories()
                    if files:
                        print("Saved trajectories:")
                        for f in files:
                            print(f"  - {f}")
                    else:
                        print("No saved trajectories")
                else:
                    print(f"Unknown command: {cmd}. Type 'help'")
                    
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
    node = TrajectoryRecorder()
    
    try:
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
