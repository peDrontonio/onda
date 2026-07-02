#!/usr/bin/env python3
"""
Render the Braco RRRPR robot using the **actual STL meshes** from the
`braco_description` URDF, in several poses.

Each link's mesh is placed in the world by:

    world_vertex = T_link(q) · ( scale · mesh_vertex  +  visual_origin )

where T_link(q) comes from the Robotics Toolbox (`robot.fkine_all(q)`), the
visual-origin offsets and the 0.001 scale are read from the URDF, and the STL
vertices are loaded with numpy-stl.

Run (inside the project venv):
    python render_urdf_meshes.py [--output DIR]
"""

import argparse
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from stl import mesh as stlmesh

from braco_rtb import robot, ik, fk_position

# ── URDF geometry: link name -> (stl file, visual-origin offset) ──────────────
MESH_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', 'ros_ws', 'src', 'braco_description', 'meshes'))
SCALE = 0.001  # URDF mesh scale (mm -> m)

LINK_VISUAL = {
    'base_link': ('base_link.stl', np.array([0.0,  0.0,   0.0])),
    'rot1_1':    ('rot1_1.stl',    np.array([0.0,  0.0,  -0.035])),
    'rot2_1':    ('rot2_1.stl',    np.array([-0.1, 0.0,  -0.1])),
    'rot3_1':    ('rot3_1.stl',    np.array([-0.18, -0.1, -0.1])),
    'prism1_1':  ('prism1_1.stl',  np.array([0.0, -0.2,  -0.1])),
    'rot4_1':    ('rot4_1.stl',    np.array([0.0, -0.365, -0.1])),
}

# Okabe–Ito, one hue per link (colour-blind safe)
LINK_COLORS = {
    'base_link': '#999999', 'rot1_1': '#0072B2', 'rot2_1': '#D55E00',
    'rot3_1':    '#009E73', 'prism1_1': '#E69F00', 'rot4_1': '#CC79A7',
}

# cache loaded triangles (in metres, link-frame incl. visual origin) per link
_TRI_CACHE = {}


def _link_triangles(link_name):
    """Return (Ntri, 3, 3) triangle vertices in the LINK frame (metres)."""
    if link_name not in _TRI_CACHE:
        fname, offset = LINK_VISUAL[link_name]
        m = stlmesh.Mesh.from_file(os.path.join(MESH_DIR, fname))
        tris = m.vectors.astype(float) * SCALE + offset   # (Ntri,3,3)
        _TRI_CACHE[link_name] = tris
    return _TRI_CACHE[link_name]


def _shade(rgb, tris_world, light=np.array([0.4, 0.4, 0.85])):
    """Return per-triangle RGBA shaded by surface-normal · light direction."""
    light = light / np.linalg.norm(light)
    v0, v1, v2 = tris_world[:, 0], tris_world[:, 1], tris_world[:, 2]
    n = np.cross(v1 - v0, v2 - v0)
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)
    intensity = 0.45 + 0.55 * np.clip(np.abs(n @ light), 0, 1)
    base = np.array(matplotlib.colors.to_rgb(rgb))
    cols = np.clip(intensity[:, None] * base, 0, 1)
    return np.hstack([cols, np.ones((len(cols), 1))])


def _render_pose(ax, q):
    Ts = robot.fkine_all(q)          # 7 frames: index 0 = base, link i -> i+1
    for i, link in enumerate(robot.links):
        if link.name not in LINK_VISUAL:
            continue
        T = Ts[i + 1].A
        tris = _link_triangles(link.name)                 # (Ntri,3,3) link frame
        flat = tris.reshape(-1, 3)                         # (Ntri*3,3)
        world = (T[:3, :3] @ flat.T).T + T[:3, 3]          # to world
        world = world.reshape(-1, 3, 3)
        poly = Poly3DCollection(world, linewidths=0)
        poly.set_facecolor(_shade(LINK_COLORS[link.name], world))
        ax.add_collection3d(poly)


def _setup_axes(ax, title):
    ax.set_xlim(-0.30, 0.30); ax.set_ylim(-0.05, 0.45); ax.set_zlim(0.0, 0.55)
    ax.set_box_aspect((0.60, 0.50, 0.55))
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.view_init(elev=20, azim=-60)
    ax.set_title(title, fontsize=10)


def _poses():
    poses = [('home  (q = 0)', np.zeros(5))]
    for label, xyz in [
        ('alvo (-0.155, 0.29, 0.36)', (-0.155, 0.290, 0.361)),
        ('alvo (0.15, 0.30, 0.25)',   (0.15, 0.30, 0.25)),
        ('alvo (-0.20, 0.28, 0.30)',  (-0.20, 0.28, 0.30)),
        ('alvo (0.10, 0.32, 0.30)',   (0.10, 0.32, 0.30)),
    ]:
        q, ok, _ = ik(xyz)
        if ok:
            poses.append((label, q))
    poses.append(('prismática estendida', np.array([0.5, 0.4, 0.3, 0.1, -0.4])))
    return poses


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', '-o', default='output/urdf_poses')
    args = ap.parse_args()
    os.makedirs(args.output, exist_ok=True)

    poses = _poses()

    # individual PNGs
    for i, (title, q) in enumerate(poses):
        fig = plt.figure(figsize=(6, 5.5))
        ax = fig.add_subplot(111, projection='3d')
        _render_pose(ax, q)
        p = fk_position(q)
        _setup_axes(ax, f'{title}\nEE = ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m')
        path = os.path.join(args.output, f'urdf_pose_{i}.png')
        fig.savefig(path, dpi=140, bbox_inches='tight'); plt.close(fig)
        print(f'  ✓ {path}')

    # montage
    n = len(poses); cols = 3; rows = (n + cols - 1) // cols
    fig = plt.figure(figsize=(cols * 5, rows * 4.6))
    fig.suptitle('Braço RRRPR — meshes do URDF (braco_description) em várias poses',
                 fontsize=15, fontweight='bold')
    for i, (title, q) in enumerate(poses):
        ax = fig.add_subplot(rows, cols, i + 1, projection='3d')
        _render_pose(ax, q)
        p = fk_position(q)
        _setup_axes(ax, f'{title}\nEE=({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})')
    montage = os.path.join(args.output, 'montage_urdf.png')
    fig.savefig(montage, dpi=130, bbox_inches='tight'); plt.close(fig)
    print(f'  ✓ {montage}')
    print('\nFeito!')


if __name__ == '__main__':
    main()
