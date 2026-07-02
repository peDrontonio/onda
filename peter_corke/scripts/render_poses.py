#!/usr/bin/env python3
"""
Render the Braco RRRPR URDF model (loaded in the Robotics Toolbox) in several
poses and save both individual PNGs and a combined montage.

Run (inside the project venv):
    python render_poses.py [--output DIR]
"""

import argparse
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from braco_rtb import robot, ik, fk_position

# name -> joint vector q (rad / m).  Some come from IK on a cartesian target.
def _pose_list():
    poses = [('home  (q = 0)', np.zeros(5))]

    targets = [
        ('alvo (-0.155, 0.29, 0.36)', (-0.155, 0.290, 0.361)),
        ('alvo (0.15, 0.30, 0.25)',   (0.15, 0.30, 0.25)),
        ('alvo (-0.20, 0.28, 0.30)',  (-0.20, 0.28, 0.30)),
        ('alvo (0.10, 0.32, 0.30)',   (0.10, 0.32, 0.30)),
    ]
    for label, xyz in targets:
        q, ok, _ = ik(xyz)
        if ok:
            poses.append((label, q))

    # explicit joint configurations
    poses.append(('juntas giradas',      np.array([1.2, 0.9, 0.6, 0.0, 0.5])))
    poses.append(('prismática estendida', np.array([0.6, 0.4, 0.3, 0.1, -0.4])))
    return poses


AZIM, ELEV = -60, 22
XLIM, YLIM, ZLIM = (-0.35, 0.35), (-0.05, 0.45), (-0.15, 0.45)


def _render_one(q, title, path):
    env = robot.plot(q, block=False)
    ax = env.ax
    ax.set_xlim(*XLIM); ax.set_ylim(*YLIM); ax.set_zlim(*ZLIM)
    ax.view_init(elev=ELEV, azim=AZIM)
    p = fk_position(q)
    ax.set_title(f'{title}\nEE = ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m', fontsize=10)
    env.fig.set_size_inches(6, 5)
    env.fig.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(env.fig)
    print(f'  ✓ {path}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', '-o', default='output/poses')
    args = ap.parse_args()
    os.makedirs(args.output, exist_ok=True)

    poses = _pose_list()
    paths = []
    for i, (title, q) in enumerate(poses):
        path = os.path.join(args.output, f'pose_{i}.png')
        _render_one(q, title, path)
        paths.append(path)

    # montage
    n = len(paths)
    cols = 3
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(cols * 5, rows * 4.2))
    fig.suptitle('Braço RRRPR (modelo URDF no Robotics Toolbox) — poses',
                 fontsize=15, fontweight='bold')
    for ax, path in zip(axes.flat, paths):
        ax.imshow(plt.imread(path)); ax.axis('off')
    for ax in axes.flat[n:]:
        ax.axis('off')
    plt.tight_layout()
    montage = os.path.join(args.output, 'montage.png')
    fig.savefig(montage, dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'  ✓ {montage}')
    print('\nFeito!')


if __name__ == '__main__':
    main()
