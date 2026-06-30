#!/usr/bin/env python3
"""
PD + gravity compensation controller for the Braco RRRPR manipulator.

Subscribes to:
  /joint_states              — current joint positions and velocities
  /braco/trajectory_command  — JointTrajectory reference (from trajectory_planner)

Publishes to:
  /effort_controller/commands — Float64MultiArray effort commands

Control law:
    τ = Kp*(q_des − q) + Kd*(q̇_des − q̇) + G(q)

Run:
    ros2 run braco_description braco_controller.py
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray


JOINT_ORDER = ['base_rot1', 'rot1_rot2', 'rot2_rot3', 'rot3_prism1', 'prism1_rot4']

# URDF masses [kg]
MASSES = [1.910661, 6.846799, 7.274379, 14.513248, 4.193506, 0.654538]
G_ACC  = 9.81  # m/s²

# PD gains
KP = np.array([120.0, 100.0, 80.0, 500.0, 30.0])
KD = np.array([ 10.0,   8.0,  6.0,  30.0,  2.0])

# Effort saturation matching URDF <limit effort="100"/>
MAX_EFFORT = 100.0


def _gravity(q):
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


class BracoController(Node):

    def __init__(self):
        super().__init__('braco_controller')

        # Current state
        self._q  = np.zeros(5)
        self._qd = np.zeros(5)
        self._joint_index = {}   # joint_name → index in JOINT_ORDER

        # Trajectory reference
        self._traj_points = []      # list of (t_from_start_sec, q, qd)
        self._traj_t0     = None    # rclpy.time.Time when trajectory was received

        # Publisher
        self._effort_pub = self.create_publisher(
            Float64MultiArray, '/effort_controller/commands', 10)

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.create_subscription(JointTrajectory, '/braco/trajectory_command',
                                 self._traj_cb, 10)

        # 100 Hz control loop
        self.create_timer(0.01, self._control_loop)
        self.get_logger().info('BracoController started (PD + gravity compensation)')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in JOINT_ORDER:
                idx = JOINT_ORDER.index(name)
                if idx not in self._joint_index:
                    self._joint_index[name] = idx
                self._q[idx]  = msg.position[i] if msg.position else 0.0
                self._qd[idx] = msg.velocity[i]  if msg.velocity  else 0.0

    def _traj_cb(self, msg: JointTrajectory):
        """Convert incoming JointTrajectory to internal list sorted by time."""
        name_to_local = {n: i for i, n in enumerate(msg.joint_names)}

        points = []
        for pt in msg.points:
            t_sec = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            q  = np.zeros(5)
            qd = np.zeros(5)
            for jname, li in name_to_local.items():
                if jname in JOINT_ORDER:
                    gi = JOINT_ORDER.index(jname)
                    q[gi]  = pt.positions[li]  if pt.positions  else 0.0
                    qd[gi] = pt.velocities[li] if pt.velocities else 0.0
            points.append((t_sec, q, qd))

        points.sort(key=lambda x: x[0])
        self._traj_points = points
        self._traj_t0     = self.get_clock().now()
        self.get_logger().info(
            f'New trajectory received: {len(points)} points, '
            f'duration={points[-1][0]:.2f}s')

    # ── Control loop ─────────────────────────────────────────────────────────

    def _interpolate_reference(self, t_elapsed):
        """Linear interpolation along trajectory points."""
        if not self._traj_points:
            return self._q.copy(), np.zeros(5)

        if t_elapsed <= self._traj_points[0][0]:
            return self._traj_points[0][1].copy(), self._traj_points[0][2].copy()

        if t_elapsed >= self._traj_points[-1][0]:
            return self._traj_points[-1][1].copy(), np.zeros(5)

        for k in range(1, len(self._traj_points)):
            t0, q0, qd0 = self._traj_points[k - 1]
            t1, q1, qd1 = self._traj_points[k]
            if t_elapsed <= t1:
                alpha = (t_elapsed - t0) / (t1 - t0)
                return (q0 + alpha * (q1 - q0),
                        qd0 + alpha * (qd1 - qd0))

        return self._traj_points[-1][1].copy(), np.zeros(5)

    def _control_loop(self):
        if self._traj_t0 is None:
            # Gravity compensation only — hold current position
            tau = _gravity(self._q)
        else:
            t_elapsed = (self.get_clock().now() - self._traj_t0).nanoseconds * 1e-9
            q_des, qd_des = self._interpolate_reference(t_elapsed)
            tau = KP * (q_des - self._q) + KD * (qd_des - self._qd) + _gravity(self._q)

        # Saturate to URDF effort limits
        tau = np.clip(tau, -MAX_EFFORT, MAX_EFFORT)

        msg = Float64MultiArray()
        msg.data = tau.tolist()
        self._effort_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BracoController()
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
