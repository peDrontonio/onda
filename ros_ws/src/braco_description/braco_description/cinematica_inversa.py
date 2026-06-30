"""
Kinematics for the Braco RRRPR manipulator.

FK is built directly from URDF joint origins — no DH approximation needed.
IK uses scipy SLSQP with multiple restarts for reliability.

Joint chain (from URDF):
  base_rot1   — revolute  Z  at (0.000,  0.000, 0.035)
  rot1_rot2   — revolute  X  at (0.100,  0.000, 0.065)  rel to parent
  rot2_rot3   — revolute  Y  at (0.080,  0.100, 0.000)
  rot3_prism1 — prismatic Y  at (-0.180, 0.100, 0.000)  range [0, 0.1 m]
  prism1_rot4 — revolute -X  at (0.000,  0.165, 0.000)  range ±60°

At q = 0 the end-effector sits at (0, 0.365, 0.1) in the world frame.
"""

import numpy as np
from scipy.optimize import minimize

# ── Joint limits ──────────────────────────────────────────────────────────────
JOINT_LIMITS = [
    (0.0,       6.283185),   # base_rot1   [0, 2π]
    (0.0,       6.283185),   # rot1_rot2   [0, 2π]
    (0.0,       6.283185),   # rot2_rot3   [0, 2π]
    (0.0,       0.1     ),   # rot3_prism1 [0, 0.1 m]
    (-1.047198, 1.047198),   # prism1_rot4 [±60°]
]

JOINT_NAMES = [
    'base_rot1', 'rot1_rot2', 'rot2_rot3', 'rot3_prism1', 'prism1_rot4'
]

# ── Rotation helpers ──────────────────────────────────────────────────────────

def _Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def _Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def _htm(xyz, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T

# ── Forward kinematics ────────────────────────────────────────────────────────

def fk(q):
    """
    Return the 4×4 end-effector homogeneous transform for joint vector q.

    q = [q1, q2, q3, d4, q5]
      q1, q2, q3, q5 : radians
      d4              : metres  (prismatic extension)
    """
    q1, q2, q3, d4, q5 = q

    T = (
        _htm([0.0,  0.0,        0.035], _Rz( q1))
      @ _htm([0.1,  0.0,        0.065], _Rx( q2))
      @ _htm([0.08, 0.1,        0.0  ], _Ry( q3))
      @ _htm([-0.18, 0.1 + d4,  0.0  ], np.eye(3))   # prismatic along Y
      @ _htm([0.0,  0.165,      0.0  ], _Rx(-q5))
    )
    return T


def fk_position(q):
    """Return (x, y, z) end-effector position."""
    return fk(q)[:3, 3]


def chain_points(q):
    """
    Return (xs, ys, zs) arrays of joint positions along the kinematic chain,
    including the base and end-effector — useful for 3-D visualisation.
    """
    q1, q2, q3, d4, q5 = q
    pts = [[0.0, 0.0, 0.0]]   # world origin

    T = np.eye(4)
    steps = [
        _htm([0.0,   0.0,       0.035], _Rz( q1)),
        _htm([0.1,   0.0,       0.065], _Rx( q2)),
        _htm([0.08,  0.1,       0.0  ], _Ry( q3)),
        _htm([-0.18, 0.1 + d4,  0.0  ], np.eye(3)),
        _htm([0.0,   0.165,     0.0  ], _Rx(-q5)),
    ]
    for step in steps:
        T = T @ step
        pts.append(T[:3, 3].tolist())

    pts = np.array(pts)
    return pts[:, 0], pts[:, 1], pts[:, 2]

# ── Inverse kinematics ────────────────────────────────────────────────────────

def _initial_guesses(x, y, z):
    """Generate diverse initial guesses spread around the likely solution."""
    q1 = np.arctan2(y, x) % (2 * np.pi)
    guesses = []
    for dq1 in [0.0, np.pi]:
        for q2 in [0.3, 0.8, 1.5]:
            for q3 in [0.3, 1.0, 2.0]:
                guesses.append([
                    (q1 + dq1) % (2 * np.pi),
                    q2, q3, 0.05,
                ])
    return guesses


def ik(target_xyz, q5=0.0, q_init=None):
    """
    Numerical IK: find q = [q1, q2, q3, d4] (q5 fixed) that places the
    end-effector at target_xyz.

    Returns
    -------
    q_sol : ndarray shape (5,)
    success : bool   — True if position error < 1 mm
    error_m : float  — residual position error in metres
    """
    target = np.array(target_xyz, dtype=float)

    bounds = JOINT_LIMITS[:4]   # q5 is held fixed

    def cost(qf):
        return float(np.sum((fk_position([*qf, q5]) - target) ** 2))

    guesses = [q_init[:4]] if q_init is not None else _initial_guesses(*target)

    best = None
    for q0 in guesses:
        res = minimize(cost, q0, method='SLSQP', bounds=bounds,
                       options={'ftol': 1e-10, 'maxiter': 2000})
        if best is None or res.fun < best.fun:
            best = res
        if best.fun < 1e-8:
            break

    q_sol = np.array([*best.x, q5])
    error_m = float(np.sqrt(best.fun))
    success = error_m < 1e-3   # < 1 mm

    return q_sol, success, error_m


# ── Demo ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    # Verify FK at home pose
    q_home = np.zeros(5)
    p_home = fk_position(q_home)
    print(f"FK at home (q=0): {p_home.round(4)}")   # expect [0, 0.365, 0.1]

    # Round-trip: FK → IK → FK
    q_true = np.array([np.deg2rad(40), np.deg2rad(30), np.deg2rad(20), 0.05, np.deg2rad(-15)])
    p_true = fk_position(q_true)
    print(f"\nTarget position (from q_true): {p_true.round(4)}")

    q_hat, ok, err = ik(p_true, q5=q_true[4])
    p_hat = fk_position(q_hat)
    print(f"IK solution found: {ok}  (position error {err*1000:.3f} mm)")
    print(f"FK at IK solution:  {p_hat.round(4)}")
    print(f"q_true (deg): {np.rad2deg(q_true).round(2)}")
    print(f"q_hat  (deg): {np.rad2deg(q_hat).round(2)}")
