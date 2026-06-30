#!/usr/bin/env python3
"""
Send a Cartesian target to the Braco manipulator via IK.

Usage:
    ros2 run braco_description send_cartesian.py <x> <y> <z>
    ros2 run braco_description send_cartesian.py 0.2 0.15 0.25 --duration 4.0

The node computes IK and publishes a JointTrajectory to
/joint_trajectory_controller/joint_trajectory, which the
joint_trajectory_controller action server listens on.
"""

import argparse
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from braco_description.cinematica_inversa import fk_position, ik, JOINT_NAMES, JOINT_LIMITS


class CartesianCommandSender(Node):

    TOPIC = '/braco/trajectory_command'

    def __init__(self, x, y, z, duration=3.0, q5=0.0):
        super().__init__('cartesian_command_sender')
        self._target = (x, y, z)
        self._duration = duration
        self._q5 = q5
        self._sent = False

        self._pub = self.create_publisher(JointTrajectory, self.TOPIC, 10)
        self.create_timer(0.5, self._send)

    def _send(self):
        if self._sent:
            rclpy.shutdown()
            return

        x, y, z = self._target
        self.get_logger().info(f'Target: x={x:.3f} y={y:.3f} z={z:.3f}')

        q, ok, err = ik((x, y, z), q5=self._q5)

        if not ok:
            self.get_logger().error(
                f'IK did not converge (error={err*1000:.1f} mm). '
                'Target may be outside workspace.'
            )
            rclpy.shutdown()
            return

        self.get_logger().info(f'IK solved (error={err*1000:.3f} mm):')
        for name, val in zip(JOINT_NAMES, q):
            if name == 'rot3_prism1':
                self.get_logger().info(f'  {name}: {val*1000:.2f} mm')
            else:
                self.get_logger().info(f'  {name}: {np.degrees(val):.2f} deg')

        # Verify with FK
        p_check = fk_position(q)
        self.get_logger().info(
            f'FK check: x={p_check[0]:.4f} y={p_check[1]:.4f} z={p_check[2]:.4f}'
        )

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(JOINT_NAMES)

        pt = JointTrajectoryPoint()
        pt.positions = q.tolist()
        pt.velocities = [0.0] * 5
        pt.time_from_start = Duration(
            sec=int(self._duration),
            nanosec=int((self._duration % 1) * 1e9)
        )
        msg.points.append(pt)

        self._pub.publish(msg)
        self.get_logger().info(f'Trajectory sent (duration {self._duration}s)')
        self._sent = True


def main():
    parser = argparse.ArgumentParser(
        description='Send a Cartesian target to the Braco manipulator'
    )
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('z', type=float)
    parser.add_argument('--duration', '-d', type=float, default=3.0)
    parser.add_argument('--rotation', '-r', type=float, default=0.0,
                        help='End-effector rotation q5 in radians')

    # strip ROS args before parsing
    args = parser.parse_args([a for a in sys.argv[1:] if not a.startswith('--ros-args')])

    rclpy.init()
    node = CartesianCommandSender(args.x, args.y, args.z, args.duration, args.rotation)
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
