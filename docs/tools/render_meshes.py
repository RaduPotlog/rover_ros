#!/usr/bin/env python3
"""Render the Rover A1 URDF meshes for the platform manual and print the overall dimensions.

Run from the repository root:

    uv run --with trimesh,matplotlib,numpy python docs/tools/render_meshes.py

Writes docs/assets/images/rover_a1_iso.png, rover_a1_top.png and rover_a1_side.png. The wheel
placement follows rover_description: wheels at (+/- wheelbase/2, +/- wheel_separation/2,
wheel_mount_point_z) in body_link, and body_link at tyre_radius - wheel_mount_point_z above the
ground (base_footprint).
"""

from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402
import yaml  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402
from PIL import Image  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
DESC = ROOT / 'rover_description'
OUT = ROOT / 'docs' / 'assets' / 'images'

WHEEL_MOUNT_POINT_Z = 0.037363  # rover_description/urdf/rover_a1/rover_a1_macro.urdf.xacro

BODY_COLOUR = (0.79, 0.82, 0.93)
WHEEL_COLOUR = (0.18, 0.18, 0.20)


def load_scene():
    cfg = yaml.safe_load((DESC / 'config' / 'wheel_01.yaml').read_text())
    base_z = cfg['tyre_radius'] - WHEEL_MOUNT_POINT_Z
    hx, hy = cfg['wheelbase'] / 2.0, cfg['wheel_separation'] / 2.0

    body = trimesh.load(DESC / 'meshes' / 'rover_a1' / 'base.stl')
    body.apply_translation([0.0, 0.0, base_z])
    parts = [(body, BODY_COLOUR)]
    for name, x, y in (('fl', hx, hy), ('fr', hx, -hy), ('rl', -hx, hy), ('rr', -hx, -hy)):
        wheel = trimesh.load(DESC / 'meshes' / 'wheel_01' / f'{name}_wheel.stl')
        wheel.apply_translation([x, y, WHEEL_MOUNT_POINT_Z + base_z])
        parts.append((wheel, WHEEL_COLOUR))
    return cfg, base_z, parts


def shade(mesh, colour):
    light = np.array([0.4, -0.5, 0.75])
    light /= np.linalg.norm(light)
    k = 0.45 + 0.55 * np.clip(mesh.face_normals @ light, 0.0, 1.0)
    return np.column_stack([np.outer(k, colour), np.ones(len(k))])


def render(parts, path, elev, azim, max_faces=60000):
    fig = plt.figure(figsize=(8, 6), dpi=220)
    ax = fig.add_subplot(projection='3d', proj_type='ortho')
    # One collection for all parts, so matplotlib depth-sorts the faces across parts.
    triangles, colours = [], []
    for mesh, colour in parts:
        if len(mesh.faces) > max_faces:
            mesh = mesh.simplify_quadric_decimation(face_count=max_faces)
        triangles.append(mesh.triangles)
        colours.append(shade(mesh, colour))
    coll = Poly3DCollection(np.concatenate(triangles), facecolors=np.concatenate(colours),
                            linewidths=0)
    ax.add_collection3d(coll)
    lo = np.min([m.bounds[0] for m, _ in parts], axis=0)
    hi = np.max([m.bounds[1] for m, _ in parts], axis=0)
    centre, span = (lo + hi) / 2.0, (hi - lo).max() / 2.0
    for setter, c in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), centre):
        setter(c - span, c + span)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    fig.savefig(path, transparent=True, bbox_inches='tight', pad_inches=0)
    plt.close(fig)
    # The 3D axes keep a square canvas: crop to the drawn pixels plus a small margin.
    img = Image.open(path)
    left, top, right, bottom = img.getchannel('A').getbbox()
    margin = 12
    img.crop((max(left - margin, 0), max(top - margin, 0),
              min(right + margin, img.width), min(bottom + margin, img.height))).save(path)


def main():
    cfg, base_z, parts = load_scene()
    lo = np.min([m.bounds[0] for m, _ in parts], axis=0)
    hi = np.max([m.bounds[1] for m, _ in parts], axis=0)
    body_lo, body_hi = parts[0][0].bounds
    print(f'base_link height above ground: {base_z:.4f} m')
    print(f'overall length x width x height: {hi[0] - lo[0]:.3f} x {hi[1] - lo[1]:.3f} x {hi[2]:.3f} m')
    print(f'body length x width: {body_hi[0] - body_lo[0]:.3f} x {body_hi[1] - body_lo[1]:.3f} m')
    print(f'body underside above ground: {body_lo[2]:.3f} m')

    OUT.mkdir(parents=True, exist_ok=True)
    render(parts, OUT / 'rover_a1_iso.png', elev=22, azim=-55)
    render(parts, OUT / 'rover_a1_top.png', elev=90, azim=-90)
    render(parts, OUT / 'rover_a1_side.png', elev=0, azim=-90)


if __name__ == '__main__':
    main()
