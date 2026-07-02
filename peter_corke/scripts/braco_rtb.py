"""
RTB (Peter Corke Robotics Toolbox) model of the Braco RRRPR manipulator.

The model is imported **directly from the project URDF**
(`ros_ws/src/braco_description/urdf/braco.urdf`), so the kinematics *and* the
dynamic parameters (link masses, centres of mass and inertia tensors) come from
the real robot — no DH approximation and no hand-tuned inertias.

It exposes the same helper API as the original `cinematica_inversa.py`
(`fk`, `fk_position`, `chain_points`, `ik`, `JOINT_NAMES`, `JOINT_LIMITS`) plus
the Robotics Toolbox dynamics used by the controller/plant:

    inertia(q)          -> M(q)              (mass matrix)
    gravload(q)         -> G(q)              (gravity torque, used by PD+G)
    nonlinear(q, qd)    -> C(q,q̇)·q̇ + G(q)   (via rne, one call)
    plant_accel(q,qd,τ) -> q̈ = M⁻¹(τ − C·q̇ − G)

Note: RTB's `Robot.coriolis()` convenience wrapper is broken for URDF-imported
models (it indexes a copied base link), so we never call it — `nonlinear()`
uses `rne(q, qd, 0)` directly, which is exact and fast.
"""

import os
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

# ── Locate and load the URDF ──────────────────────────────────────────────────

_HERE = os.path.dirname(os.path.abspath(__file__))
URDF_DIR = os.environ.get(
    "BRACO_URDF_DIR",
    os.path.normpath(os.path.join(
        _HERE, "..", "..", "ros_ws", "src", "braco_description", "urdf")),
)
URDF_FILE = "braco.urdf"

G_ACC = 9.81  # m/s²


def _load_robot():
    links, name, _urdf_str, _path = rtb.Robot.URDF_read(URDF_FILE, tld=URDF_DIR)
    robot = rtb.Robot(links, name=name)
    robot.gravity = [0.0, 0.0, -G_ACC]
    return robot


robot = _load_robot()

# ── Joint metadata (same as cinematica_inversa) ───────────────────────────────

JOINT_LIMITS = [
    (0.0,       6.283185),   # base_rot1   [0, 2π]  revolute Z
    (0.0,       6.283185),   # rot1_rot2   [0, 2π]  revolute X
    (0.0,       6.283185),   # rot2_rot3   [0, 2π]  revolute Y
    (0.0,       0.1     ),   # rot3_prism1 [0, 0.1 m] prismatic Y
    (-1.047198, 1.047198),   # prism1_rot4 [±60°]   revolute X
]

JOINT_NAMES = [
    'base_rot1', 'rot1_rot2', 'rot2_rot3', 'rot3_prism1', 'prism1_rot4'
]

# ── Forward kinematics ────────────────────────────────────────────────────────

def fk(q):
    """Return the 4×4 end-effector homogeneous transform for joint vector q."""
    return robot.fkine(q).A


def fk_position(q):
    """Return (x, y, z) end-effector position."""
    return robot.fkine(q).t


def chain_points(q):
    """
    Return (xs, ys, zs) arrays of the frame origins along the kinematic chain
    (base → end-effector), for 3-D visualisation.
    """
    Ts = robot.fkine_all(q)          # SE3 array, one per link (incl. base)
    pts = np.array([T.t for T in Ts])
    return pts[:, 0], pts[:, 1], pts[:, 2]

# ── Inverse kinematics (position only, 5-DOF arm) ─────────────────────────────

def ik(target_xyz, q5=0.0, q_init=None):
    """
    Numerical IK via RTB Levenberg–Marquardt (`ikine_LM`), position only
    (orientation is left free — the arm has 5 DOF for a 3-DOF position task).

    Returns
    -------
    q_sol   : ndarray shape (5,)
    success : bool   — True if position error < 1 mm
    error_m : float  — residual position error in metres
    """
    target = np.array(target_xyz, dtype=float)
    q0 = np.asarray(q_init, dtype=float) if q_init is not None else np.zeros(robot.n)

    sol = robot.ikine_LM(
        SE3(*target), q0=q0,
        mask=[1, 1, 1, 0, 0, 0],   # constrain x, y, z only
        joint_limits=True,
    )
    q_sol = np.asarray(sol.q, dtype=float)
    error_m = float(np.linalg.norm(fk_position(q_sol) - target))
    success = bool(sol.success) and error_m < 1e-3
    return q_sol, success, error_m

# ── Dynamics (from the URDF, via Robotics Toolbox) ────────────────────────────

def inertia(q):
    """Joint-space mass matrix M(q)  (5×5)."""
    return robot.inertia(q)


def gravload(q):
    """Gravity torque vector G(q)  — used by the PD + gravity-comp controller."""
    return robot.gravload(q)


def nonlinear(q, qd):
    """
    Combined Coriolis/centrifugal + gravity torques  C(q,q̇)·q̇ + G(q).
    Computed with a single recursive Newton–Euler call (rne with q̈ = 0).
    """
    return robot.rne(q, qd, np.zeros(robot.n))


def plant_accel(q, qd, tau):
    """
    Forward dynamics of the plant:  q̈ = M(q)⁻¹ · [τ − C(q,q̇)·q̇ − G(q)].
    """
    M = robot.inertia(q)
    h = robot.rne(q, qd, np.zeros(robot.n))
    return np.linalg.solve(M, tau - h)


# ── Self-test ─────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    q_home = np.zeros(robot.n)
    print(robot)
    print(f"FK at home (q=0): {fk_position(q_home).round(4)}   (expect [0, 0.365, 0.1])")
    print(f"masses (kg)     : {[round(l.m, 3) for l in robot.links if l.m is not None]}")
    print(f"gravload(0)     : {gravload(q_home).round(3)}")

    q_sol, ok, err = ik((-0.155, 0.290, 0.361))
    print(f"\nIK (-0.155, 0.290, 0.361): success={ok}  error={err*1000:.3f} mm")
    print(f"q_sol deg/mm    : "
          f"{[round(np.degrees(q_sol[0]),2), round(np.degrees(q_sol[1]),2), round(np.degrees(q_sol[2]),2), round(q_sol[3]*1000,1), round(np.degrees(q_sol[4]),2)]}")
