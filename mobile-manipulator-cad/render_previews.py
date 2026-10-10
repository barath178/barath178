"""Render PNG previews straight from the generated URDF + STL meshes.

This is also a check that the URDF joint origins/axes put every mesh in the
right place. Run after build_models.py:  python render_previews.py
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "ros2_ws" / "src"
OUT = ROOT / "previews"


def read_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    n = int.from_bytes(data[80:84], "little")
    rec = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([
        ("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return rec["v"].astype(float)


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def origin(el):
    T = np.eye(4)
    o = el.find("origin")
    if o is not None:
        T[:3, :3] = rpy_mat(*map(float, o.get("rpy", "0 0 0").split()))
        T[:3, 3] = list(map(float, o.get("xyz", "0 0 0").split()))
    return T


def axis_rot(a, q):
    a = np.asarray(a) / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
    return T


def link_poses(urdf: ET.Element, q: dict):
    joints = urdf.findall("joint")
    children = {j.find("child").get("link") for j in joints}
    root = next(l.get("name") for l in urdf.findall("link") if l.get("name") not in children)
    T = {root: np.eye(4)}
    pending = list(joints)
    while pending:
        for j in list(pending):
            p = j.find("parent").get("link")
            if p not in T:
                continue
            M = T[p] @ origin(j)
            ax = j.find("axis")
            a = list(map(float, ax.get("xyz").split())) if ax is not None else [1, 0, 0]
            v = q.get(j.get("name"), 0.0)
            if j.get("type") in ("revolute", "continuous"):
                M = M @ axis_rot(a, v)
            elif j.get("type") == "prismatic":
                M = M.copy()
                M[:3, 3] += M[:3, :3] @ (np.asarray(a) * v)
            T[j.find("child").get("link")] = M
            pending.remove(j)
    return T


def robot_tris(pkg: str, name: str, q: dict, base=np.eye(4)):
    urdf = ET.parse(SRC / pkg / "urdf" / f"{name}.urdf").getroot()
    T = link_poses(urdf, q)
    out = []
    for link in urdf.findall("link"):
        vis = link.find("visual")
        if vis is None:
            continue
        mesh = vis.find("geometry/mesh").get("filename").split("/meshes/")[1]
        rgba = list(map(float, vis.find("material/color").get("rgba").split()))
        tri = read_stl(SRC / pkg / "meshes" / mesh)
        M = base @ T[link.get("name")] @ origin(vis)
        tri = tri @ M[:3, :3].T + M[:3, 3]
        out.append((tri, rgba))
    return out


def box_tris(x0, x1, y0, y1, z0, z1):
    v = np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)])
    f = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return np.array([[v[a], v[b], v[c]] for a, b, c, d in f] + [[v[a], v[c], v[d]] for a, b, c, d in f])


def draw(groups, path, elev=22, azim=-55, title=""):
    light = np.array([0.4, -0.5, 0.8])
    light /= np.linalg.norm(light)
    fig = plt.figure(figsize=(8, 8), dpi=110)
    ax = fig.add_subplot(projection="3d")
    tris, cols = [], []
    for tri, rgba in groups:
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        shade = 0.45 + 0.55 * np.abs(n @ light)
        tris.append(tri)
        cols.append(np.clip(np.outer(shade, rgba[:3]), 0, 1))
    # one collection so matplotlib depth-sorts every triangle together
    tri_all, col_all = np.vstack(tris), np.vstack(cols)
    ax.add_collection3d(Poly3DCollection(tri_all, facecolors=np.c_[col_all, np.ones(len(col_all))],
                                         linewidths=0))
    allv = [tri_all.reshape(-1, 3)]
    v = np.vstack(allv)
    c, r = (v.max(0) + v.min(0)) / 2, (v.max(0) - v.min(0)).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev, azim)
    ax.set_axis_off()
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def main():
    OUT.mkdir(exist_ok=True)
    ur5_q = {"shoulder_lift_joint": -1.57, "elbow_joint": 1.2, "wrist_1_joint": -1.2,
             "wrist_2_joint": -1.57, "gripper_left_joint": 0.03, "gripper_right_joint": 0.03}
    mm_q = {"joint2": -1.0, "joint3": 0.3, "joint4": 0.7,
            "gripper_left_joint": 0.01, "gripper_right_joint": 0.01}
    draw(robot_tris("ur5_description", "ur5", {}), OUT / "ur5_zero_pose.png", title="UR5 - all joints = 0")
    draw(robot_tris("ur5_description", "ur5", ur5_q), OUT / "ur5_ready_pose.png", title="UR5 - ready pose")
    draw(robot_tris("tb3_omx_description", "tb3_omx", {}), OUT / "tb3_omx_zero_pose.png", azim=-60,
         title="TurtleBot3 Waffle Pi + OpenMANIPULATOR-X - all joints = 0")
    draw(robot_tris("tb3_omx_description", "tb3_omx", mm_q), OUT / "tb3_omx_ready_pose.png", azim=-60,
         title="TurtleBot3 Waffle Pi + OpenMANIPULATOR-X - ready pose")

    base = np.eye(4)
    base[:3, 3] = (1.0, -0.9, 0.0)
    cell = robot_tris("ur5_description", "ur5", ur5_q) + robot_tris("tb3_omx_description", "tb3_omx", mm_q, base)
    wood, card, obst = (0.72, 0.56, 0.38, 1), (0.80, 0.62, 0.38, 1), (0.90, 0.45, 0.10, 1)
    cell += [(box_tris(0.35, 0.95, -0.40, 0.40, 0.69, 0.72), wood),
             (box_tris(1.30, 1.70, -0.25, 0.25, 0.20, 0.22), wood),
             (box_tris(0.34, 0.46, 0.59, 0.71, 0, 1.0), obst),
             (box_tris(-1.0, -0.6, 0.6, 1.0, 0, 0.5), obst),
             (box_tris(0.45, 0.75, -1.15, -0.85, 0, 0.3), obst),
             (box_tris(-1.2, 1.9, -1.4, 1.2, -0.01, 0.0), (0.85, 0.85, 0.85, 1))]
    cell += [(box_tris(lx - 0.02, lx + 0.02, ly - 0.02, ly + 0.02, 0, 0.69), wood)
             for lx in (0.38, 0.92) for ly in (-0.37, 0.37)]
    cell += [(box_tris(lx - 0.01, lx + 0.01, ly - 0.01, ly + 0.01, 0, 0.20), wood)
             for lx in (1.31, 1.69) for ly in (-0.24, 0.24)]
    for bx, by, bz in [(0.50, -0.20, 0.72), (0.62, 0.05, 0.72), (0.75, 0.22, 0.72), (1.15, -0.40, 0)]:
        cell.append((box_tris(bx - 0.03, bx + 0.03, by - 0.03, by + 0.03, bz, bz + 0.06), card))
    draw(cell, OUT / "material_handling_cell.png", elev=28, azim=-50, title="Material-handling cell")
    print("previews written to", OUT)


if __name__ == "__main__":
    main()
