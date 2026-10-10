"""CAD + ROS 2 generator for the project
"Collision-Free Motion Planning for Mobile Manipulator During Material Handling".

  Fixed manipulator  : UR5 (6-DOF, Universal Robots) + parallel gripper
  Mobile manipulator : TurtleBot3 Waffle Pi + OpenMANIPULATOR-X

For each robot this script writes
  solidworks/<robot>/parts/*.STEP        one file per part (open in SolidWorks)
  solidworks/<robot>/<robot>_assembly.STEP   full assembly, every URDF link
                                             is one sub-assembly
  ros2_ws/src/<pkg>/                     ROS 2 package laid out like the
      CMakeLists.txt  package.xml        SolidWorks-to-URDF exporter output
      export.log
      config/   launch/   meshes/   urdf/   textures/   worlds/

Kinematics (joint origins and axes) follow the official ROS descriptions
(ur_description, turtlebot3_description, open_manipulator_x_description).
Part shapes are simplified envelopes; dimensions are approximate.

STEP files are in millimetres, STL meshes and URDF in metres.

Run:  python build_models.py      (pip install cadquery)
"""

from __future__ import annotations

import math
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from xml.sax.saxutils import escape

import cadquery as cq
import numpy as np
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Trsf

ROOT = Path(__file__).resolve().parent
SW_OUT = ROOT / "solidworks"
ROS_OUT = ROOT / "ros2_ws" / "src"
MM = 1000.0
PI = math.pi

# ================================================================ math helpers


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def tf(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    """URDF origin -> 4x4 transform (rpy = fixed X, Y, Z)."""
    T = np.eye(4)
    T[:3, :3] = rot_z(rpy[2]) @ rot_y(rpy[1]) @ rot_x(rpy[0])
    T[:3, 3] = xyz
    return T


def axis_angle(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
    return T


def loc(T):
    """4x4 in metres -> cadquery Location in millimetres."""
    t = gp_Trsf()
    R, p = T[:3, :3], T[:3, 3] * MM
    t.SetValues(*R[0], p[0], *R[1], p[1], *R[2], p[2])
    return cq.Location(t)


def S(v):
    return v * MM


# ================================================================ shape helpers (inputs in metres)


def box(x0, x1, y0, y1, z0, z1):
    return (
        cq.Workplane("XY")
        .box(S(x1 - x0), S(y1 - y0), S(z1 - z0))
        .translate((S(x0 + x1) / 2, S(y0 + y1) / 2, S(z0 + z1) / 2))
    )


def cyl(axis, r, a0, a1, c=(0, 0)):
    """Cylinder radius r along `axis` from a0 to a1. c = the other two
    coordinates of the axis line, in x, y, z order."""
    if axis == "z":
        wp = cq.Workplane("XY", origin=(S(c[0]), S(c[1]), S(a0)))
    elif axis == "y":
        wp = cq.Workplane("XZ", origin=(S(c[0]), S(a1), S(c[1])))
    else:
        wp = cq.Workplane("YZ", origin=(S(a0), S(c[0]), S(c[1])))
    return wp.circle(S(r)).extrude(S(a1 - a0))


def sphere(r, c):
    return cq.Workplane("XY").sphere(S(r)).translate(tuple(S(v) for v in c))


def bar_xz(p0, p1, width, y0, y1):
    """Rounded flat bar in the XZ plane from p0=(x,z) to p1, between y0 and y1."""
    (x0, z0), (x1, z1) = p0, p1
    L = math.hypot(x1 - x0, z1 - z0)
    ang = math.degrees(math.atan2(z1 - z0, x1 - x0))
    return (
        cq.Workplane("XZ", origin=(0, S(y1), 0))
        .center(S(x0 + x1) / 2, S(z0 + z1) / 2)
        .slot2D(S(L + width), S(width), ang)
        .extrude(S(y1 - y0))
    )


def xm430(horn_axis, horn_c, body_dir):
    """DYNAMIXEL XM430 envelope (46.5 x 28.5 x 34 mm) with its horn.
    horn_axis 'y' or 'z'; body_dir is a unit axis vector the body extends along."""
    L, W, D, off = 0.0465, 0.0285, 0.034, 0.01125
    c = np.array(horn_c, float)
    d = np.array(body_dir, float)
    a, b = c - d * off, c + d * (L - off)
    lo, hi = np.minimum(a, b), np.maximum(a, b)
    half = {"y": (W / 2, D / 2, W / 2), "z": (W / 2, W / 2, D / 2)}[horn_axis]
    for i in range(3):
        if abs(d[i]) < 1e-9:
            lo[i], hi[i] = c[i] - half[i], c[i] + half[i]
    body = box(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])
    if horn_axis == "z":
        horn = cyl("z", 0.011, c[2] + D / 2 - 0.001, c[2] + D / 2 + 0.003, (c[0], c[1]))
    else:
        horn = cyl("y", 0.011, c[1] + D / 2 - 0.001, c[1] + D / 2 + 0.003, (c[0], c[2]))
    return body.union(horn)


RGBA = {
    "ur_blue": (0.32, 0.55, 0.78, 1),
    "ur_grey": (0.80, 0.82, 0.84, 1),
    "ur_dark": (0.20, 0.22, 0.25, 1),
    "black": (0.12, 0.12, 0.13, 1),
    "plate": (0.26, 0.26, 0.29, 1),
    "metal": (0.75, 0.75, 0.78, 1),
    "tire": (0.05, 0.05, 0.05, 1),
    "pcb_green": (0.10, 0.45, 0.20, 1),
    "pcb_red": (0.70, 0.12, 0.12, 1),
    "battery": (0.20, 0.30, 0.60, 1),
    "lidar": (0.25, 0.25, 0.28, 1),
    "pedestal": (0.45, 0.47, 0.50, 1),
    "wood": (0.72, 0.56, 0.38, 1),
    "cardboard": (0.80, 0.62, 0.38, 1),
    "obstacle": (0.90, 0.45, 0.10, 1),
    "floor": (0.85, 0.85, 0.85, 1),
}


def ccol(name):
    return cq.Color(*RGBA[name])


# ================================================================ robot data model


@dataclass
class Part:
    name: str
    shape: cq.Workplane  # in the owning link's frame, millimetres
    color: str


@dataclass
class Link:
    name: str
    parts: list[Part] = field(default_factory=list)
    mass: float = 0.0
    collision: tuple | None = None  # None -> mesh; ("cylinder", r, len, xyz, rpy) / ("sphere", r, xyz) / ("box", size, xyz)


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    xyz: tuple = (0, 0, 0)
    rpy: tuple = (0, 0, 0)
    axis: tuple = (0, 0, 1)
    lower: float = 0.0
    upper: float = 0.0
    effort: float = 0.0
    velocity: float = 0.0


@dataclass
class Robot:
    name: str          # robot name in URDF
    package: str       # ROS 2 package name
    title: str         # human title
    links: list[Link]
    joints: list[Joint]
    home: dict         # joint -> initial value (Gazebo + preview)
    controllers: dict  # controller name -> list of joints (JointTrajectoryController)


# ================================================================ UR5 (fixed manipulator)
# ur_description ur5 kinematics (CB3).


def build_ur5() -> Robot:
    RB, RW = 0.060, 0.045
    L = []

    L.append(Link("world"))
    L.append(Link("pedestal_link", [
        Part("pedestal_column", box(-0.10, 0.10, -0.10, 0.10, 0.0, 0.68), "pedestal"),
        Part("pedestal_top_plate", box(-0.12, 0.12, -0.12, 0.12, 0.68, 0.70), "metal"),
        Part("pedestal_foot_plate", box(-0.20, 0.20, -0.20, 0.20, 0.0, 0.015), "metal"),
    ], 25.0))
    L.append(Link("base_link", [
        Part("ur5_base_flange", cyl("z", 0.0745, 0, 0.012), "ur_grey"),
        Part("ur5_base_housing", cyl("z", RB, 0.012, 0.030), "ur_grey"),
    ], 4.0))
    L.append(Link("shoulder_link", [
        Part("ur5_shoulder_pan_motor", cyl("z", RB, -0.059, 0.060), "ur_blue"),
        Part("ur5_shoulder_lift_motor", cyl("y", RB, 0.0, 0.135, (0, 0)), "ur_blue"),
    ], 3.7))
    L.append(Link("upper_arm_link", [
        Part("ur5_upper_arm_lift_cap", cyl("y", RB, 0.0, 0.070, (0, 0)), "ur_grey"),
        Part("ur5_upper_arm_tube", cyl("z", 0.044, 0.0, 0.425, (0, 0.035)), "ur_grey"),
        Part("ur5_elbow_motor", cyl("y", RB, -0.1197, 0.070, (0, 0.425)), "ur_blue"),
    ], 8.393))
    L.append(Link("forearm_link", [
        Part("ur5_forearm_elbow_cap", cyl("y", 0.050, -0.060, 0.0, (0, 0)), "ur_grey"),
        Part("ur5_forearm_tube", cyl("z", 0.036, 0.0, 0.39225, (0, -0.030)), "ur_grey"),
        Part("ur5_wrist_1_motor", cyl("y", RW, -0.060, 0.0, (0, 0.39225)), "ur_blue"),
    ], 2.275))
    L.append(Link("wrist_1_link", [
        Part("ur5_wrist_1_cap", cyl("y", RW, 0.0, 0.050, (0, 0)), "ur_grey"),
        Part("ur5_wrist_2_motor", cyl("z", RW, -0.045, 0.045, (0, 0.093)), "ur_blue"),
    ], 1.219))
    L.append(Link("wrist_2_link", [
        Part("ur5_wrist_2_cap", cyl("z", RW, 0.045, 0.060, (0, 0)), "ur_grey"),
        Part("ur5_wrist_3_motor", cyl("y", RW, -0.045, 0.050, (0, 0.09465)), "ur_blue"),
    ], 1.219))
    L.append(Link("wrist_3_link", [
        Part("ur5_tool_flange", cyl("y", 0.032, 0.050, 0.0823, (0, 0)), "ur_dark"),
    ], 0.1879))
    L.append(Link("tool0"))
    # simple parallel gripper for material handling, in tool0 frame (x points out of the flange)
    L.append(Link("gripper_base_link", [
        Part("gripper_adapter", cyl("x", 0.032, 0.0, 0.010, (0, 0)), "ur_dark"),
        Part("gripper_body", box(0.010, 0.050, -0.062, 0.062, -0.022, 0.022), "black"),
    ], 0.25))
    L.append(Link("gripper_left_finger", [
        Part("gripper_finger_left", box(0.0, 0.065, 0.0, 0.012, -0.012, 0.012), "metal"),
    ], 0.03))
    L.append(Link("gripper_right_finger", [
        Part("gripper_finger_right", box(0.0, 0.065, -0.012, 0.0, -0.012, 0.012), "metal"),
    ], 0.03))
    L.append(Link("grasp_point"))

    big = dict(lower=-2 * PI, upper=2 * PI, effort=150.0, velocity=3.15)
    small = dict(lower=-2 * PI, upper=2 * PI, effort=28.0, velocity=3.2)
    J = [
        Joint("world_joint", "fixed", "world", "pedestal_link"),
        Joint("pedestal_joint", "fixed", "pedestal_link", "base_link", (0, 0, 0.70)),
        Joint("shoulder_pan_joint", "revolute", "base_link", "shoulder_link",
              (0, 0, 0.089159), axis=(0, 0, 1), **big),
        Joint("shoulder_lift_joint", "revolute", "shoulder_link", "upper_arm_link",
              (0, 0.13585, 0), (0, PI / 2, 0), (0, 1, 0), **big),
        Joint("elbow_joint", "revolute", "upper_arm_link", "forearm_link",
              (0, -0.1197, 0.425), axis=(0, 1, 0), lower=-PI, upper=PI, effort=150.0, velocity=3.15),
        Joint("wrist_1_joint", "revolute", "forearm_link", "wrist_1_link",
              (0, 0, 0.39225), (0, PI / 2, 0), (0, 1, 0), **small),
        Joint("wrist_2_joint", "revolute", "wrist_1_link", "wrist_2_link",
              (0, 0.093, 0), axis=(0, 0, 1), **small),
        Joint("wrist_3_joint", "revolute", "wrist_2_link", "wrist_3_link",
              (0, 0, 0.09465), axis=(0, 1, 0), **small),
        Joint("tool0_joint", "fixed", "wrist_3_link", "tool0", (0, 0.0823, 0), (0, 0, PI / 2)),
        Joint("gripper_mount_joint", "fixed", "tool0", "gripper_base_link"),
        Joint("gripper_left_joint", "prismatic", "gripper_base_link", "gripper_left_finger",
              (0.050, 0.012, 0), axis=(0, 1, 0), lower=0.0, upper=0.035, effort=40.0, velocity=0.1),
        Joint("gripper_right_joint", "prismatic", "gripper_base_link", "gripper_right_finger",
              (0.050, -0.012, 0), axis=(0, -1, 0), lower=0.0, upper=0.035, effort=40.0, velocity=0.1),
        Joint("grasp_point_joint", "fixed", "gripper_base_link", "grasp_point", (0.095, 0, 0)),
    ]
    arm = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    home = {"shoulder_lift_joint": -1.57, "elbow_joint": 1.2, "wrist_1_joint": -1.2,
            "wrist_2_joint": -1.57, "gripper_left_joint": 0.03, "gripper_right_joint": 0.03}
    return Robot("ur5", "ur5_description", "UR5 fixed manipulator", L, J, home,
                 {"arm_controller": arm,
                  "gripper_controller": ["gripper_left_joint", "gripper_right_joint"]})


# ================================================================ TurtleBot3 Waffle Pi + OpenMANIPULATOR-X
# turtlebot3_waffle_pi.urdf + turtlebot3_manipulation + open_manipulator_x.

TB3_CX = -0.064
PLATE_Z = (0.000, 0.045, 0.091)
PLATE_T = 0.003
TOP_Z = PLATE_Z[2] + PLATE_T


def waffle_plate(z):
    w = 0.266
    plate = box(TB3_CX - w / 2, TB3_CX + w / 2, -w / 2, w / 2, z, z + PLATE_T)
    pts = [(S(TB3_CX + i * 0.03), S(j * 0.03)) for i in range(-3, 4) for j in range(-3, 4)]
    cut = (cq.Workplane("XY", origin=(0, 0, S(z) - 1)).pushPoints(pts)
           .rect(S(0.016), S(0.016)).extrude(S(PLATE_T) + 2))
    return plate.cut(cut)


def build_tb3_omx() -> Robot:
    L = [Link("base_footprint")]

    standoffs = None
    for sx in (-0.12, 0.12):
        for sy in (-0.12, 0.12):
            s = cyl("z", 0.0035, 0.0, TOP_Z, (TB3_CX + sx, sy))
            standoffs = s if standoffs is None else standoffs.union(s)
    L.append(Link("base_link", [
        Part("waffle_plate_bottom", waffle_plate(PLATE_Z[0]), "plate"),
        Part("waffle_plate_middle", waffle_plate(PLATE_Z[1]), "plate"),
        Part("waffle_plate_top", waffle_plate(PLATE_Z[2]), "plate"),
        Part("standoffs_x4", standoffs, "metal"),
        Part("wheel_motor_left_xm430", box(-0.023, 0.023, 0.060, 0.120, 0.006, 0.040), "black"),
        Part("wheel_motor_right_xm430", box(-0.023, 0.023, -0.120, -0.060, 0.006, 0.040), "black"),
        Part("lipo_battery", box(TB3_CX - 0.050, TB3_CX + 0.050, -0.017, 0.017, 0.004, 0.034), "battery"),
        Part("opencr_board", box(TB3_CX - 0.052, TB3_CX + 0.052, -0.052, 0.052, 0.048, 0.054), "pcb_red"),
        Part("raspberry_pi", box(TB3_CX - 0.073, TB3_CX + 0.013, -0.028, 0.028, 0.064, 0.080), "pcb_green"),
    ], 1.3729, ("box", (0.266, 0.266, 0.094), (TB3_CX, 0, 0.047))))

    def wheel(name, side):
        return Link(name, [
            Part(f"tire_{side}", cyl("y", 0.033, -0.009, 0.009, (0, 0)), "tire"),
            Part(f"wheel_hub_{side}", cyl("y", 0.020, -0.0105, 0.0105, (0, 0)), "metal"),
        ], 0.0285, ("cylinder", 0.033, 0.018, (0, 0, 0), (PI / 2, 0, 0)))

    L.append(wheel("wheel_left_link", "left"))
    L.append(wheel("wheel_right_link", "right"))

    def caster(name, side):
        return Link(name, [
            Part(f"ball_caster_holder_{side}", cyl("z", 0.009, 0.0, 0.004), "metal"),
            Part(f"ball_caster_ball_{side}", sphere(0.006, (0, 0, 0)), "metal"),
        ], 0.005, ("sphere", 0.006, (0, 0, 0)))

    L.append(caster("caster_back_left_link", "left"))
    L.append(caster("caster_back_right_link", "right"))
    L.append(Link("base_scan", [
        Part("lds01_body", box(-0.034, 0.0355, -0.04775, 0.04775, -0.028, -0.008), "lidar"),
        Part("lds01_turret", cyl("z", 0.034, -0.008, 0.0115), "lidar"),
    ], 0.114))
    L.append(Link("camera_link", [
        Part("raspberry_pi_camera", box(-0.004, 0.004, -0.0125, 0.0125, -0.0125, 0.0125), "pcb_green"),
    ], 0.005))

    # --- OpenMANIPULATOR-X (link frames from open_manipulator_x.urdf.xacro)
    L.append(Link("link1", [
        Part("omx_base_plate", box(-0.030, 0.050, -0.030, 0.030, 0.0, 0.004), "black"),
        Part("omx_joint1_xm430", xm430("z", (0.012, 0, 0.017), (-1, 0, 0)), "black"),
    ], 0.079))
    L.append(Link("link2", [
        Part("omx_link2_bracket", box(-0.0142, 0.0142, -0.017, 0.017, 0.0, 0.025), "metal"),
        Part("omx_joint2_xm430", xm430("y", (0, 0, 0.0595), (0, 0, -1)), "black"),
    ], 0.098))
    L.append(Link("link3", [
        Part("omx_link3_plate_left", bar_xz((0, 0), (0.024, 0.128), 0.024, 0.0185, 0.0215), "metal"),
        Part("omx_link3_plate_right", bar_xz((0, 0), (0.024, 0.128), 0.024, -0.0215, -0.0185), "metal"),
        Part("omx_joint3_xm430", xm430("y", (0.024, 0, 0.128), (0, 0, -1)), "black"),
    ], 0.138))
    L.append(Link("link4", [
        Part("omx_link4_plate_left", bar_xz((0, 0), (0.124, 0), 0.024, 0.0215, 0.0245), "metal"),
        Part("omx_link4_plate_right", bar_xz((0, 0), (0.124, 0), 0.024, -0.0245, -0.0215), "metal"),
        Part("omx_joint4_xm430", xm430("y", (0.124, 0, 0), (-1, 0, 0)), "black"),
    ], 0.133))
    L.append(Link("link5", [
        Part("omx_link5_plate_left", bar_xz((0, 0), (0.045, 0), 0.024, 0.0185, 0.0215), "metal"),
        Part("omx_link5_plate_right", bar_xz((0, 0), (0.045, 0), 0.024, -0.0215, -0.0185), "metal"),
        Part("omx_gripper_xm430", box(0.030, 0.060, -0.0175, 0.0175, -0.0143, 0.0143), "black"),
        Part("omx_gripper_rail", box(0.060, 0.070, -0.035, 0.035, -0.012, 0.012), "black"),
    ], 0.143))
    L.append(Link("gripper_left_link", [
        Part("omx_finger_left", box(-0.0117, 0.0443, -0.009, -0.001, -0.010, 0.010), "black"),
    ], 0.010))
    L.append(Link("gripper_right_link", [
        Part("omx_finger_right", box(-0.0117, 0.0443, 0.001, 0.009, -0.010, 0.010), "black"),
    ], 0.010))
    L.append(Link("end_effector_link"))

    J = [
        Joint("base_joint", "fixed", "base_footprint", "base_link", (0, 0, 0.010)),
        Joint("wheel_left_joint", "continuous", "base_link", "wheel_left_link",
              (0, 0.144, 0.023), axis=(0, 1, 0), effort=10.0, velocity=10.0),
        Joint("wheel_right_joint", "continuous", "base_link", "wheel_right_link",
              (0, -0.144, 0.023), axis=(0, 1, 0), effort=10.0, velocity=10.0),
        Joint("caster_back_left_joint", "fixed", "base_link", "caster_back_left_link", (-0.177, 0.064, -0.004)),
        Joint("caster_back_right_joint", "fixed", "base_link", "caster_back_right_link", (-0.177, -0.064, -0.004)),
        Joint("scan_joint", "fixed", "base_link", "base_scan", (0.034, 0, TOP_Z + 0.028)),
        Joint("camera_joint", "fixed", "base_link", "camera_link", (0.073, 0, 0.080)),
        Joint("omx_mount_joint", "fixed", "base_link", "link1", (-0.092, 0, TOP_Z)),
        Joint("joint1", "revolute", "link1", "link2", (0.012, 0, 0.034), axis=(0, 0, 1),
              lower=-2.83, upper=2.83, effort=1.0, velocity=4.8),
        Joint("joint2", "revolute", "link2", "link3", (0, 0, 0.0595), axis=(0, 1, 0),
              lower=-1.79, upper=1.57, effort=1.0, velocity=4.8),
        Joint("joint3", "revolute", "link3", "link4", (0.024, 0, 0.128), axis=(0, 1, 0),
              lower=-0.94, upper=1.38, effort=1.0, velocity=4.8),
        Joint("joint4", "revolute", "link4", "link5", (0.124, 0, 0), axis=(0, 1, 0),
              lower=-1.79, upper=2.04, effort=1.0, velocity=4.8),
        Joint("gripper_left_joint", "prismatic", "link5", "gripper_left_link", (0.0817, 0.021, 0),
              axis=(0, 1, 0), lower=-0.010, upper=0.019, effort=1.0, velocity=4.8),
        Joint("gripper_right_joint", "prismatic", "link5", "gripper_right_link", (0.0817, -0.021, 0),
              axis=(0, -1, 0), lower=-0.010, upper=0.019, effort=1.0, velocity=4.8),
        Joint("end_effector_joint", "fixed", "link5", "end_effector_link", (0.126, 0, 0)),
    ]
    home = {"joint2": -1.0, "joint3": 0.3, "joint4": 0.7,
            "gripper_left_joint": 0.01, "gripper_right_joint": 0.01}
    return Robot("tb3_omx", "tb3_omx_description",
                 "TurtleBot3 Waffle Pi + OpenMANIPULATOR-X mobile manipulator", L, J, home,
                 {"arm_controller": ["joint1", "joint2", "joint3", "joint4"],
                  "gripper_controller": ["gripper_left_joint", "gripper_right_joint"]})


# ================================================================ geometry processing


def link_compound(link: Link, scale=1.0):
    solids = []
    for p in link.parts:
        solids.extend(p.shape.vals())
    comp = cq.Compound.makeCompound(solids)
    return comp.scale(scale) if scale != 1.0 else comp


def inertial(link: Link):
    """Mass properties from the CAD volume, scaled to the link mass."""
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(link_compound(link).wrapped, props)
    vol = props.Mass() * 1e-9  # mm^3 -> m^3
    com = props.CentreOfMass()
    M = props.MatrixOfInertia()
    rho = link.mass / vol
    I = np.array([[M.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)]) * 1e-15 * rho
    I = 0.5 * (I + I.T)
    I[np.diag_indices(3)] = np.maximum(np.diag(I), 1e-7)
    return (com.X() / MM, com.Y() / MM, com.Z() / MM), I


def fk(robot: Robot, q: dict):
    """World transform of every link."""
    T = {}
    children = {j.parent: [] for j in robot.joints}
    for j in robot.joints:
        children.setdefault(j.parent, []).append(j)
    root = robot.links[0].name
    T[root] = np.eye(4)
    stack = [root]
    while stack:
        p = stack.pop()
        for j in children.get(p, []):
            M = T[p] @ tf(j.xyz, j.rpy)
            v = q.get(j.name, 0.0)
            if j.type in ("revolute", "continuous"):
                M = M @ axis_angle(j.axis, v)
            elif j.type == "prismatic":
                M = M.copy()
                M[:3, 3] += M[:3, :3] @ (np.asarray(j.axis, float) * v)
            T[j.child] = M
            stack.append(j.child)
    return T


def robot_assembly(robot: Robot, q: dict, name=None):
    """SolidWorks assembly: one sub-assembly per URDF link, placed by FK."""
    asm = cq.Assembly(name=name or robot.name)
    T = fk(robot, q)
    for link in robot.links:
        if not link.parts:
            continue
        sub = cq.Assembly(name=link.name)
        for p in link.parts:
            sub.add(p.shape, name=p.name, color=ccol(p.color))
        asm.add(sub, name=link.name, loc=loc(T[link.name]))
    return asm


# ================================================================ URDF / xacro writers


def f(v):
    return f"{0.0 if abs(v) < 1e-12 else v:.6g}"


def vec(v):
    return " ".join(f(x) for x in v)


def urdf_body(robot: Robot) -> str:
    out = []
    for link in robot.links:
        if not link.parts:
            out.append(f'  <link name="{link.name}"/>\n')
            continue
        com, I = inertial(link)
        rgba = RGBA[link.parts[0].color]
        mesh = f"package://{robot.package}/meshes/{link.name}.STL"
        col = link.collision
        if col is None:
            cgeo = f'<mesh filename="{mesh}"/>'
            corg = '<origin xyz="0 0 0" rpy="0 0 0"/>'
        elif col[0] == "cylinder":
            cgeo = f'<cylinder radius="{f(col[1])}" length="{f(col[2])}"/>'
            corg = f'<origin xyz="{vec(col[3])}" rpy="{vec(col[4])}"/>'
        elif col[0] == "sphere":
            cgeo = f'<sphere radius="{f(col[1])}"/>'
            corg = f'<origin xyz="{vec(col[2])}" rpy="0 0 0"/>'
        else:
            cgeo = f'<box size="{vec(col[1])}"/>'
            corg = f'<origin xyz="{vec(col[2])}" rpy="0 0 0"/>'
        out.append(f"""  <link name="{link.name}">
    <inertial>
      <origin xyz="{vec(com)}" rpy="0 0 0"/>
      <mass value="{f(link.mass)}"/>
      <inertia ixx="{f(I[0,0])}" ixy="{f(I[0,1])}" ixz="{f(I[0,2])}" iyy="{f(I[1,1])}" iyz="{f(I[1,2])}" izz="{f(I[2,2])}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{mesh}"/>
      </geometry>
      <material name="{link.name}_material">
        <color rgba="{vec(rgba)}"/>
      </material>
    </visual>
    <collision>
      {corg}
      <geometry>
        {cgeo}
      </geometry>
    </collision>
  </link>
""")
    for j in robot.joints:
        s = f"""  <joint name="{j.name}" type="{j.type}">
    <origin xyz="{vec(j.xyz)}" rpy="{vec(j.rpy)}"/>
    <parent link="{j.parent}"/>
    <child link="{j.child}"/>
"""
        if j.type != "fixed":
            s += f'    <axis xyz="{vec(j.axis)}"/>\n'
        if j.type in ("revolute", "prismatic"):
            s += f'    <limit lower="{f(j.lower)}" upper="{f(j.upper)}" effort="{f(j.effort)}" velocity="{f(j.velocity)}"/>\n'
        elif j.type == "continuous":
            s += f'    <limit effort="{f(j.effort)}" velocity="{f(j.velocity)}"/>\n'
        if j.type != "fixed":
            s += '    <dynamics damping="0.1" friction="0.0"/>\n'
        s += "  </joint>\n"
        out.append(s)
    return "".join(out)


def ros2_control_block(robot: Robot) -> str:
    def jnt(name, cmd):
        init = robot.home.get(name, 0.0)
        return f"""    <joint name="{name}">
      <command_interface name="{cmd}"/>
      <state_interface name="position">
        <param name="initial_value">{f(init)}</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
"""
    body = ""
    for j in robot.joints:
        if j.type == "continuous":
            body += jnt(j.name, "velocity")
        elif j.type in ("revolute", "prismatic"):
            body += jnt(j.name, "position")
    return f"""
  <!-- ros2_control: Gazebo (gz_ros2_control) hardware interface -->
  <ros2_control name="GazeboSimSystem" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
{body}  </ros2_control>

  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(arg controllers_file)</parameters>
    </plugin>
  </gazebo>
"""


def gazebo_extras(robot: Robot) -> str:
    if robot.name != "tb3_omx":
        return ""
    return """
  <!-- wheel / caster friction -->
  <gazebo reference="wheel_left_link"><mu1>1.0</mu1><mu2>1.0</mu2><kp>500000.0</kp><kd>10.0</kd></gazebo>
  <gazebo reference="wheel_right_link"><mu1>1.0</mu1><mu2>1.0</mu2><kp>500000.0</kp><kd>10.0</kd></gazebo>
  <gazebo reference="caster_back_left_link"><mu1>0.0</mu1><mu2>0.0</mu2></gazebo>
  <gazebo reference="caster_back_right_link"><mu1>0.0</mu1><mu2>0.0</mu2></gazebo>

  <!-- LDS-01 360 deg lidar, published on /scan through ros_gz_bridge -->
  <gazebo reference="base_scan">
    <sensor name="lds01" type="gpu_lidar">
      <topic>scan</topic>
      <gz_frame_id>base_scan</gz_frame_id>
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>5</update_rate>
      <lidar>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>0.0</min_angle>
            <max_angle>6.28319</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.12</min>
          <max>3.5</max>
          <resolution>0.015</resolution>
        </range>
      </lidar>
    </sensor>
  </gazebo>
"""


def write_urdf_files(robot: Robot, pkg: Path):
    body = urdf_body(robot)
    header = f"<!-- {robot.title}\n     Generated by build_models.py (CAD -> URDF). Meshes in package://{robot.package}/meshes -->\n"
    (pkg / "urdf" / f"{robot.name}.urdf").write_text(
        f'<?xml version="1.0" encoding="utf-8"?>\n{header}<robot name="{robot.name}">\n{body}</robot>\n'
    )
    (pkg / "urdf" / f"{robot.name}.urdf.xacro").write_text(
        f'<?xml version="1.0" encoding="utf-8"?>\n{header}'
        f'<robot name="{robot.name}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
        '  <!-- path to config/ros2_controllers.yaml, passed in by launch/gazebo.launch.py -->\n'
        '  <xacro:arg name="controllers_file" default=""/>\n\n'
        f"{body}{ros2_control_block(robot)}{gazebo_extras(robot)}</robot>\n"
    )


# ================================================================ ROS 2 package files


def cmakelists(robot: Robot):
    return f"""cmake_minimum_required(VERSION 3.8)
project({robot.package})

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf textures worlds
  DESTINATION share/${{PROJECT_NAME}}
)

ament_package()
"""


def package_xml(robot: Robot):
    deps = ["robot_state_publisher", "joint_state_publisher", "joint_state_publisher_gui",
            "rviz2", "xacro", "ros_gz_sim", "ros_gz_bridge", "gz_ros2_control",
            "controller_manager", "joint_state_broadcaster", "joint_trajectory_controller"]
    if robot.name == "tb3_omx":
        deps.append("diff_drive_controller")
    dep_xml = "\n".join(f"  <exec_depend>{d}</exec_depend>" for d in deps)
    return f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{robot.package}</name>
  <version>1.0.0</version>
  <description>{robot.title}: URDF, meshes, RViz and Gazebo launch files generated from the SolidWorks/STEP model.</description>
  <maintainer email="barath178@users.noreply.github.com">barath178</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

{dep_xml}

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""


def controllers_yaml(robot: Robot):
    lines = ["controller_manager:", "  ros__parameters:", "    update_rate: 100", "",
             "    joint_state_broadcaster:", "      type: joint_state_broadcaster/JointStateBroadcaster", ""]
    for c in robot.controllers:
        lines += [f"    {c}:", "      type: joint_trajectory_controller/JointTrajectoryController", ""]
    if robot.name == "tb3_omx":
        lines += ["    diff_drive_controller:", "      type: diff_drive_controller/DiffDriveController", ""]
    for c, joints in robot.controllers.items():
        lines += [f"{c}:", "  ros__parameters:", "    joints:"]
        lines += [f"      - {j}" for j in joints]
        lines += ["    command_interfaces:", "      - position", "    state_interfaces:",
                  "      - position", "      - velocity", "    allow_partial_joints_goal: true", ""]
    if robot.name == "tb3_omx":
        lines += [
            "diff_drive_controller:", "  ros__parameters:",
            "    left_wheel_names: [\"wheel_left_joint\"]",
            "    right_wheel_names: [\"wheel_right_joint\"]",
            "    wheel_separation: 0.287", "    wheel_radius: 0.033",
            "    base_frame_id: base_footprint", "    odom_frame_id: odom",
            "    enable_odom_tf: true", "    publish_rate: 50.0",
            "    linear.x.max_velocity: 0.26", "    linear.x.min_velocity: -0.26",
            "    angular.z.max_velocity: 1.82", "    angular.z.min_velocity: -1.82", ""]
    return "\n".join(lines)


def joint_names_yaml(robot: Robot):
    names = [j.name for j in robot.joints if j.type != "fixed"]
    return "controller_joint_names: [" + ", ".join(f"'{n}'" for n in names) + "]\n"


def rviz_config(robot: Robot, fixed_frame: str, scan: bool):
    scan_display = """
    - Class: rviz_default_plugins/LaserScan
      Enabled: true
      Name: LaserScan
      Size (m): 0.02
      Style: Points
      Topic:
        Value: /scan
        Reliability Policy: Best Effort
      Color Transformer: FlatColor
      Color: 255; 0; 0
""" if scan else ""
    return f"""Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Enabled: true
      Name: Grid
      Cell Size: 0.1
      Plane Cell Count: 40
    - Class: rviz_default_plugins/RobotModel
      Enabled: true
      Name: RobotModel
      Description Source: Topic
      Description Topic:
        Value: /robot_description
      Visual Enabled: true
      Collision Enabled: false
    - Class: rviz_default_plugins/TF
      Enabled: false
      Name: TF
      Show Names: true
      Marker Scale: 0.3{scan_display}
  Global Options:
    Fixed Frame: {fixed_frame}
    Background Color: 48; 48; 48
    Frame Rate: 30
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: {2.2 if robot.name == 'ur5' else 0.9}
      Focal Point:
        X: 0
        Y: 0
        Z: {0.8 if robot.name == 'ur5' else 0.15}
      Pitch: 0.45
      Yaw: 0.8
"""


DISPLAY_LAUNCH = '''"""RViz: view {title} and move every joint with sliders.

    ros2 launch {pkg} display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory("{pkg}")
    robot_description = xacro.process_file(
        os.path.join(pkg, "urdf", "{name}.urdf.xacro")
    ).toxml()

    gui = LaunchConfiguration("gui")
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="joint_state_publisher_gui sliders"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{{"robot_description": robot_description}}], output="screen"),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             condition=IfCondition(gui)),
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             condition=UnlessCondition(gui)),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", os.path.join(pkg, "config", "display.rviz")]),
    ])
'''

GAZEBO_LAUNCH = '''"""Gazebo (gz sim) : spawn {title} in the material-handling world
with ros2_control controllers.

    ros2 launch {pkg} gazebo.launch.py
    ros2 launch {pkg} gazebo.launch.py rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory("{pkg}")
    controllers = os.path.join(pkg, "config", "ros2_controllers.yaml")
    world = os.path.join(pkg, "worlds", "material_handling.sdf")
    robot_description = xacro.process_file(
        os.path.join(pkg, "urdf", "{name}.urdf.xacro"),
        mappings={{"controllers_file": controllers}},
    ).toxml()

    # let Gazebo resolve package://{pkg}/meshes/...
    resource_path = os.path.dirname(pkg)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={{"gz_args": f"-r -v 3 {{world}}", "on_exit_shutdown": "true"}}.items(),
    )
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               parameters=[{{"robot_description": robot_description, "use_sim_time": True}}],
               output="screen")
    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-topic", "robot_description", "-name", "{name}",
                            "-x", "{x}", "-y", "{y}", "-z", "{z}", "-Y", "{yaw}"])
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                  arguments={bridge_args})

    def spawner(name):
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "--controller-manager", "/controller_manager"])

    jsb = spawner("joint_state_broadcaster")
    rest = [spawner(c) for c in {controllers}]

    rviz = Node(package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("rviz")),
                arguments=["-d", os.path.join(pkg, "config", "gazebo.rviz")],
                parameters=[{{"use_sim_time": True}}])

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_path),
        gz_sim, rsp, spawn, bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=rest)),
        rviz,
    ])
'''


def world_sdf():
    def static_box(name, size, pose, rgba):
        return f"""    <model name="{name}">
      <static>true</static>
      <pose>{vec(pose)} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{vec(size)}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{vec(size)}</size></box></geometry>
          <material><ambient>{vec(rgba)}</ambient><diffuse>{vec(rgba)}</diffuse></material></visual>
      </link>
    </model>
"""

    def payload(name, pose):
        s, m = 0.06, 0.05
        i = m * s * s / 6
        rgba = RGBA["cardboard"]
        return f"""    <model name="{name}">
      <pose>{vec(pose)} 0 0 0</pose>
      <link name="link">
        <inertial><mass>{m}</mass><inertia><ixx>{i:.3e}</ixx><iyy>{i:.3e}</iyy><izz>{i:.3e}</izz></inertia></inertial>
        <collision name="c"><geometry><box><size>{s} {s} {s}</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><box><size>{s} {s} {s}</size></box></geometry>
          <material><ambient>{vec(rgba)}</ambient><diffuse>{vec(rgba)}</diffuse></material></visual>
      </link>
    </model>
"""
    wood, obst = RGBA["wood"], RGBA["obstacle"]
    tz = 0.72
    models = "".join([
        static_box("work_table", (0.6, 0.8, 0.03), (0.65, 0, tz - 0.015), wood),
        *[static_box(f"work_table_leg_{i}", (0.04, 0.04, tz - 0.03), (lx, ly, (tz - 0.03) / 2), wood)
          for i, (lx, ly) in enumerate([(0.38, -0.37), (0.38, 0.37), (0.92, -0.37), (0.92, 0.37)])],
        payload("payload_box_1", (0.50, -0.20, tz + 0.03)),
        payload("payload_box_2", (0.62, 0.05, tz + 0.03)),
        payload("payload_box_3", (0.75, 0.22, tz + 0.03)),
        static_box("obstacle_column", (0.12, 0.12, 1.0), (0.40, 0.65, 0.5), obst),
        static_box("drop_shelf", (0.40, 0.50, 0.02), (1.50, 0, 0.21), wood),
        *[static_box(f"drop_shelf_leg_{i}", (0.02, 0.02, 0.20), (lx, ly, 0.10), wood)
          for i, (lx, ly) in enumerate([(1.31, -0.24), (1.31, 0.24), (1.69, -0.24), (1.69, 0.24)])],
        payload("payload_box_4", (1.15, -0.40, 0.03)),
        static_box("obstacle_cabinet", (0.40, 0.40, 0.50), (-0.80, 0.80, 0.25), obst),
        static_box("obstacle_crate", (0.30, 0.30, 0.30), (0.60, -1.00, 0.15), obst),
    ])
    return f"""<?xml version="1.0" ?>
<!-- Material-handling cell: work table with payloads, a drop shelf and obstacles.
     UR5 pedestal stands at the origin, the mobile manipulator starts at (1.0, -0.9). -->
<sdf version="1.8">
  <world name="material_handling">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material></visual>
      </link>
    </model>

{models}  </world>
</sdf>
"""


def export_log(robot: Robot, part_count: int):
    lines = [f"CAD -> URDF export log : {robot.title}",
             f"robot name     : {robot.name}", f"ROS 2 package  : {robot.package}",
             f"parts exported : {part_count} STEP files", "", "LINKS"]
    for link in robot.links:
        if link.parts:
            com, I = inertial(link)
            lines.append(f"  {link.name:24s} mass {link.mass:8.4f} kg  com ({vec(com)})  "
                         f"mesh meshes/{link.name}.STL  parts: {', '.join(p.name for p in link.parts)}")
        else:
            lines.append(f"  {link.name:24s} (frame only, no geometry)")
    lines += ["", "JOINTS"]
    for j in robot.joints:
        lim = f"  limits [{f(j.lower)}, {f(j.upper)}]" if j.type in ("revolute", "prismatic") else ""
        lines.append(f"  {j.name:24s} {j.type:10s} {j.parent} -> {j.child}  xyz ({vec(j.xyz)}) "
                     f"rpy ({vec(j.rpy)}) axis ({vec(j.axis)}){lim}")
    return "\n".join(lines) + "\n"


SPAWN = {"ur5": (0, 0, 0, 0), "tb3_omx": (1.0, -0.9, 0.0, 0.0)}


def write_ros_package(robot: Robot):
    pkg = ROS_OUT / robot.package
    if pkg.exists():
        shutil.rmtree(pkg)
    for d in ("config", "launch", "meshes", "urdf", "textures", "worlds"):
        (pkg / d).mkdir(parents=True)

    for link in robot.links:
        if link.parts:
            link_compound(link, 1 / MM).exportStl(
                str(pkg / "meshes" / f"{link.name}.STL"),
                tolerance=0.0002, angularTolerance=0.2, ascii=False, relative=False)

    write_urdf_files(robot, pkg)
    (pkg / "CMakeLists.txt").write_text(cmakelists(robot))
    (pkg / "package.xml").write_text(package_xml(robot))
    (pkg / "config" / f"joint_names_{robot.name}.yaml").write_text(joint_names_yaml(robot))
    (pkg / "config" / "ros2_controllers.yaml").write_text(controllers_yaml(robot))
    root = robot.links[0].name
    is_mobile = robot.name == "tb3_omx"
    (pkg / "config" / "display.rviz").write_text(rviz_config(robot, root, False))
    (pkg / "config" / "gazebo.rviz").write_text(rviz_config(robot, "odom" if is_mobile else root, is_mobile))
    (pkg / "textures" / "README.txt").write_text(
        "Put image textures (.png/.jpg) for the meshes here. The URDF uses plain colours, so this folder is empty.\n")
    (pkg / "worlds" / "material_handling.sdf").write_text(world_sdf())

    fmt = dict(title=robot.title, pkg=robot.package, name=robot.name)
    (pkg / "launch" / "display.launch.py").write_text(DISPLAY_LAUNCH.format(**fmt))
    x, y, z, yaw = SPAWN[robot.name]
    bridge = ['"/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"']
    if is_mobile:
        bridge.append('"/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"')
    ctrl = list(robot.controllers) + (["diff_drive_controller"] if is_mobile else [])
    (pkg / "launch" / "gazebo.launch.py").write_text(GAZEBO_LAUNCH.format(
        **fmt, x=x, y=y, z=z, yaw=yaw, bridge_args="[" + ", ".join(bridge) + "]",
        controllers=repr(ctrl)))
    return pkg


# ================================================================ SolidWorks (STEP) export


def write_solidworks(robot: Robot, folder_name: str):
    folder = SW_OUT / folder_name
    if folder.exists():
        shutil.rmtree(folder)
    (folder / "parts").mkdir(parents=True)
    n = 0
    for link in robot.links:
        for p in link.parts:
            cq.exporters.export(p.shape, str(folder / "parts" / f"{p.name}.STEP"))
            n += 1
    robot_assembly(robot, {}, name=f"{folder_name}_assembly").export(
        str(folder / f"{folder_name}_assembly.STEP"))
    return n


def cell_assembly(ur5: Robot, mm: Robot):
    asm = cq.Assembly(name="Material_Handling_Cell")
    asm.add(box(-1.2, 1.9, -1.4, 1.2, -0.010, 0.0), name="floor", color=ccol("floor"))
    asm.add(robot_assembly(ur5, ur5.home, "UR5"), name="UR5")
    x, y, z, yaw = SPAWN["tb3_omx"]
    asm.add(robot_assembly(mm, mm.home, "TB3_OpenManipulatorX"), name="TB3_OpenManipulatorX",
            loc=loc(tf((x, y, z), (0, 0, yaw))))
    tz = 0.72
    table = box(0.35, 0.95, -0.40, 0.40, tz - 0.03, tz)
    for lx in (0.38, 0.92):
        for ly in (-0.37, 0.37):
            table = table.union(box(lx - 0.02, lx + 0.02, ly - 0.02, ly + 0.02, 0, tz - 0.03))
    asm.add(table, name="work_table", color=ccol("wood"))
    for i, (bx, by, bz) in enumerate([(0.50, -0.20, tz), (0.62, 0.05, tz), (0.75, 0.22, tz), (1.15, -0.40, 0)]):
        asm.add(box(bx - 0.03, bx + 0.03, by - 0.03, by + 0.03, bz, bz + 0.06),
                name=f"payload_box_{i + 1}", color=ccol("cardboard"))
    shelf = box(1.30, 1.70, -0.25, 0.25, 0.20, 0.22)
    for lx in (1.31, 1.69):
        for ly in (-0.24, 0.24):
            shelf = shelf.union(box(lx - 0.01, lx + 0.01, ly - 0.01, ly + 0.01, 0, 0.20))
    asm.add(shelf, name="drop_shelf", color=ccol("wood"))
    asm.add(box(0.34, 0.46, 0.59, 0.71, 0, 1.0), name="obstacle_column", color=ccol("obstacle"))
    asm.add(box(-1.0, -0.6, 0.6, 1.0, 0, 0.5), name="obstacle_cabinet", color=ccol("obstacle"))
    asm.add(box(0.45, 0.75, -1.15, -0.85, 0, 0.3), name="obstacle_crate", color=ccol("obstacle"))
    return asm


def main():
    ur5, mm = build_ur5(), build_tb3_omx()
    for robot, folder in ((ur5, "UR5_Fixed_Manipulator"), (mm, "TB3_OpenManipulatorX")):
        n = write_solidworks(robot, folder)
        pkg = write_ros_package(robot)
        (pkg / "export.log").write_text(export_log(robot, n))
        print(f"{robot.title}: {n} parts -> {SW_OUT / folder}, ROS 2 package -> {pkg}")
    cell_assembly(ur5, mm).export(str(SW_OUT / "Material_Handling_Cell_assembly.STEP"))
    print("cell assembly written")


if __name__ == "__main__":
    main()
