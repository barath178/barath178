#!/usr/bin/env python3
"""
Parametric generator: TurtleBot3 (Waffle Pi style) + OpenMANIPULATOR-X mobile manipulator.

Edit the PARAMETERS block below, then run:
    pip install cadquery
    python generate.py

Outputs (relative to the TB3_OpenManipulatorX/ folder):
    SolidWorks/Parts/*.STEP                    every individual part (opens in SolidWorks 2025)
    SolidWorks/Assembly/*.STEP                 full robot + base / arm sub-assemblies
    SolidWorks/Parameters/*                    parameter table + SolidWorks Global Variables file
    URDF/tb3_omx_description/                  ROS 2 description package (meshes, xacro, urdf, launch)

All CAD is modelled in millimetres. URDF uses metres (meshes are scaled 0.001).
Arm kinematics follow the official ROBOTIS OpenMANIPULATOR-X description.
"""
import csv
import math
import os
import shutil

import cadquery as cq
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

# =============================================================================
# PARAMETERS  (mm, kg, rad)
# =============================================================================
P = {
    # ---- Waffle plates ----
    "plate_length": 266.0,      # along X
    "plate_width": 250.0,       # along Y
    "plate_thickness": 3.0,
    "plate_corner_radius": 12.0,
    "plate_center_x": -64.0,    # plate centre relative to wheel axle (base_link)
    "plate_hole_size": 11.0,    # square waffle holes
    "plate_hole_pitch": 18.0,
    "plate_hole_nx": 12,
    "plate_hole_ny": 11,
    "standoff_height": 45.0,
    "standoff_across_flats": 7.0,
    # ---- Drive ----
    "wheel_radius": 33.0,
    "wheel_width": 18.0,
    "wheel_separation": 288.0,  # centre-to-centre of the wheels
    "tire_thickness": 7.0,
    # ---- Dynamixel XM430 servo (used for wheels and all arm joints) ----
    "servo_width": 28.5,
    "servo_height": 46.5,
    "servo_depth": 34.0,
    "servo_horn_offset": 11.25,  # horn centre from the top end of the servo
    "servo_horn_radius": 11.0,
    "servo_horn_thickness": 3.0,
    # ---- Casters ----
    "caster_x": -177.0,
    "caster_y": 64.0,
    "caster_ball_radius": 8.0,
    # ---- Electronics / sensors ----
    "battery_length": 89.0, "battery_width": 35.0, "battery_height": 27.0,
    "opencr_length": 105.0, "opencr_width": 75.0,
    "rpi_length": 85.0, "rpi_width": 56.0,
    "lidar_length": 95.5, "lidar_width": 69.5, "lidar_height": 39.5,
    "lidar_x": -140.0,
    "camera_x": 55.0,
    # ---- OpenMANIPULATOR-X kinematics (ROBOTIS values) ----
    "arm_mount_x": 10.0,        # arm base position on the top plate
    "arm_flange_thickness": 3.0,
    "j1_x": 12.0, "j1_z": 17.0,
    "j2_z": 59.5,
    "j3_x": 24.0, "j3_z": 128.0,
    "j4_x": 124.0,
    "grip_x": 81.7, "grip_y": 21.0,
    "ee_x": 126.0,
    "arm_bracket_width": 24.0,
    "arm_bracket_thickness": 2.5,
    "finger_length": 48.0,
    # ---- Joint limits ----
    "j1_lower": -math.pi * 0.9, "j1_upper": math.pi * 0.9,
    "j2_lower": -math.pi * 0.57, "j2_upper": math.pi * 0.5,
    "j3_lower": -math.pi * 0.3, "j3_upper": math.pi * 0.44,
    "j4_lower": -math.pi * 0.57, "j4_upper": math.pi * 0.65,
    "grip_lower": -0.010, "grip_upper": 0.019,
    "arm_effort": 1.0, "arm_velocity": 4.8,
    # ---- Masses (kg) ----
    "m_base": 1.60, "m_wheel": 0.0285, "m_caster": 0.005, "m_lidar": 0.114, "m_camera": 0.035,
    "m_link1": 0.0794, "m_link2": 0.0985, "m_link3": 0.1385, "m_link4": 0.1324,
    "m_link5": 0.1438, "m_finger": 0.0100,
}

# Derived values
T_PL = P["plate_thickness"]
WHEEL_Y = P["wheel_separation"] / 2.0
WHEEL_Z = P["wheel_radius"] - 10.0          # base_link sits 10 mm above the ground (TurtleBot3)
GROUND_Z = -10.0
Z_BOTTOM = WHEEL_Z + P["servo_width"] / 2.0  # underside of the bottom plate rests on the wheel servos
Z_MIDDLE = Z_BOTTOM + T_PL + P["standoff_height"]
Z_TOP = Z_MIDDLE + T_PL + P["standoff_height"]
Z_TOP_SURFACE = Z_TOP + T_PL
CASTER_H = Z_BOTTOM - GROUND_Z
ARM_MOUNT_Z = Z_TOP_SURFACE + P["arm_flange_thickness"] + (P["servo_depth"] - P["j1_z"])

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SW_DIR = os.path.join(ROOT, "SolidWorks")
PKG = "tb3_omx_description"
PKG_DIR = os.path.join(ROOT, "URDF", PKG)

# Colours (r, g, b)
C = {
    "plate": (0.15, 0.15, 0.15), "standoff": (0.75, 0.75, 0.78), "servo": (0.08, 0.08, 0.08),
    "horn": (0.70, 0.70, 0.72), "tire": (0.05, 0.05, 0.05), "rim": (0.85, 0.85, 0.85),
    "caster": (0.6, 0.6, 0.62), "battery": (0.1, 0.25, 0.7), "pcb": (0.0, 0.45, 0.2),
    "lidar": (0.2, 0.2, 0.22), "bracket": (0.82, 0.82, 0.85), "gripper": (0.9, 0.9, 0.9),
}


# =============================================================================
# Helpers
# =============================================================================
def T(x=0, y=0, z=0, rx=0, ry=0, rz=0):
    """Location: translate, then rotate about X, Y, Z (intrinsic, degrees). v' = Rx(Ry(Rz v)) + t."""
    loc = cq.Location(cq.Vector(x, y, z))
    if rx:
        loc = loc * cq.Location(cq.Vector(), cq.Vector(1, 0, 0), rx)
    if ry:
        loc = loc * cq.Location(cq.Vector(), cq.Vector(0, 1, 0), ry)
    if rz:
        loc = loc * cq.Location(cq.Vector(), cq.Vector(0, 0, 1), rz)
    return loc


def box(lx, ly, lz, x=0, y=0, z=0):
    """Box with its centre at (x, y, z)."""
    return cq.Workplane("XY").box(lx, ly, lz).translate((x, y, z))


def cyl(r, h, x=0, y=0, z0=0):
    """Z-axis cylinder from z0 to z0+h."""
    return cq.Workplane("XY").circle(r).extrude(h).translate((x, y, z0))


def slot_plate(p1, p2, width, t, y0, lighten=True):
    """Flat link plate in the XZ plane joining p1=(x,z) and p2=(x,z); occupies y in [y0, y0+t]."""
    dx, dz = p2[0] - p1[0], p2[1] - p1[1]
    dist = math.hypot(dx, dz)
    ang = math.degrees(math.atan2(dz, dx))
    mx, mz = (p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0
    wp = cq.Workplane("XY").center(mx, mz).slot2D(dist + width, width, ang).extrude(t)
    if lighten and dist > 2.2 * width:
        cut = (cq.Workplane("XY").center(mx, mz)
               .slot2D(dist - 1.4 * width, width * 0.45, ang).extrude(t))
        wp = wp.cut(cut)
    for (px, pz) in (p1, p2):  # bearing / horn holes
        wp = wp.cut(cq.Workplane("XY").center(px, pz).circle(3.0).extrude(t))
    # XY -> XZ: local y -> global z, extrusion -> -y
    wp = wp.rotate((0, 0, 0), (1, 0, 0), 90).translate((0, y0 + t, 0))
    return wp


# =============================================================================
# Part models (each in its own part coordinate frame)
# =============================================================================
def part_waffle_plate():
    L, W = P["plate_length"], P["plate_width"]
    plate = (cq.Workplane("XY").rect(L, W).extrude(T_PL)
             .edges("|Z").fillet(P["plate_corner_radius"]))
    holes = (cq.Workplane("XY").rarray(P["plate_hole_pitch"], P["plate_hole_pitch"],
                                       P["plate_hole_nx"], P["plate_hole_ny"])
             .rect(P["plate_hole_size"], P["plate_hole_size"]).extrude(T_PL))
    plate = plate.cut(holes)
    # standoff mounting holes
    for (x, y) in standoff_xy():
        plate = plate.cut(cyl(1.6, T_PL, x - P["plate_center_x"], y))
    return plate


def standoff_xy():
    cx = P["plate_center_x"]
    hx = P["plate_length"] / 2.0 - 8.0
    hy = P["plate_width"] / 2.0 - 8.0
    return [(cx - hx, hy), (cx - hx, -hy), (cx + hx, hy), (cx + hx, -hy), (cx, hy), (cx, -hy)]


def part_standoff():
    s = cq.Workplane("XY").polygon(6, P["standoff_across_flats"] / math.cos(math.pi / 6)).extrude(P["standoff_height"])
    return s.cut(cyl(1.5, P["standoff_height"]))


def part_xm430():
    """Dynamixel XM430. Origin = horn centre on the output face, +Z = output axis."""
    w, h, d = P["servo_width"], P["servo_height"], P["servo_depth"]
    yc = -(h / 2.0 - P["servo_horn_offset"])
    body = box(w, h, d, 0, yc, -d / 2.0).edges("|Z").fillet(2.5)
    body = body.faces(">Z").edges().chamfer(0.8)
    horn = cyl(P["servo_horn_radius"], P["servo_horn_thickness"])
    horn = horn.cut(cyl(1.2, P["servo_horn_thickness"]))
    for i in range(4):
        a = math.radians(45 + 90 * i)
        horn = horn.cut(cyl(1.1, P["servo_horn_thickness"], 8 * math.cos(a), 8 * math.sin(a)))
    idler = cyl(6.0, 1.0, 0, 0, -d - 1.0)
    # cable connectors
    conn = box(10, 5, 6, 6, yc - h / 2.0 - 2.5 + 0.01, -d / 2.0).union(box(10, 5, 6, -6, yc - h / 2.0 - 2.5 + 0.01, -d / 2.0))
    return body.union(horn).union(idler).union(conn)


def part_wheel():
    """Wheel + tyre, axis +Z, centred at origin."""
    r, w, tt = P["wheel_radius"], P["wheel_width"], P["tire_thickness"]
    tire = cyl(r, w, z0=-w / 2.0).cut(cyl(r - tt, w, z0=-w / 2.0)).edges().fillet(2.0)
    rim = cyl(r - tt + 0.01, w * 0.55, z0=-w * 0.275)
    for i in range(6):
        a = math.radians(60 * i)
        rim = rim.cut(cyl(4.5, w, 16.5 * math.cos(a), 16.5 * math.sin(a), -w / 2.0))
    hub = cyl(9.0, w, z0=-w / 2.0).cut(cyl(2.5, w, z0=-w / 2.0))
    return tire.union(rim).union(hub)


def part_caster():
    """Ball caster. Origin = top mount face, extends down by CASTER_H."""
    rb = P["caster_ball_radius"]
    mount = box(30, 30, 3, 0, 0, -1.5).edges("|Z").fillet(4)
    body_h = CASTER_H - 3 - rb
    body = cyl(rb + 2.5, body_h, z0=-3 - body_h)
    ball = cq.Workplane("XY").sphere(rb).translate((0, 0, -CASTER_H + rb))
    return mount.union(body).union(ball)


def part_battery():
    return box(P["battery_length"], P["battery_width"], P["battery_height"], 0, 0,
               P["battery_height"] / 2.0).edges().fillet(2.0)


def _board(L, W, comps):
    pcb = box(L, W, 1.6, 0, 0, 5.8).edges("|Z").fillet(3)
    for sx in (-1, 1):
        for sy in (-1, 1):
            pcb = pcb.union(cyl(2.5, 5.0, sx * (L / 2 - 4), sy * (W / 2 - 4)))
            pcb = pcb.cut(cyl(1.4, 6.6, sx * (L / 2 - 4), sy * (W / 2 - 4)))
    for (lx, ly, lz, x, y) in comps:
        pcb = pcb.union(box(lx, ly, lz, x, y, 6.6 + lz / 2.0))
    return pcb


def part_opencr():
    L, W = P["opencr_length"], P["opencr_width"]
    return _board(L, W, [(14, 14, 1.8, 0, 0), (12, 9, 5, L / 2 - 6, 15), (14, 10, 10, -L / 2 + 7, -20),
                         (30, 6, 8, 0, W / 2 - 4), (30, 6, 8, 0, -W / 2 + 4), (8, 8, 9, -20, 20)])


def part_rpi():
    L, W = P["rpi_length"], P["rpi_width"]
    return _board(L, W, [(14, 14, 1.2, -10, 0), (17, 13, 16, L / 2 - 9, 18), (17, 13, 16, L / 2 - 9, 0),
                         (21, 16, 13.5, L / 2 - 10, -19), (50, 5, 8.5, -10, W / 2 - 3.5)])


def part_lidar():
    L, W, H = P["lidar_length"], P["lidar_width"], P["lidar_height"]
    base = box(L, W, 20, 0, 0, 10).edges("|Z").fillet(10)
    turret = cyl(W / 2.0 - 2.0, H - 20, 0, 0, 20)
    window = cyl(W / 2.0 + 1, 6, 0, 0, 26).cut(cyl(W / 2.0 - 4.0, 6, 0, 0, 26))
    return base.union(turret).cut(window)


def part_camera():
    """Pi camera on an L bracket. Origin = bracket base centre, lens looks +X."""
    base = box(20, 28, 2, 0, 0, 1)
    upright = box(2, 28, 30, 9, 0, 15)
    pcb = box(1, 25, 24, 10.5, 0, 18)
    lens = cyl(4.0, 5.0).rotate((0, 0, 0), (0, 1, 0), 90).translate((11, 0, 20))
    return base.union(upright).union(pcb).union(lens)


def part_arm_flange():
    f = box(60, 50, P["arm_flange_thickness"], 0, 0, P["arm_flange_thickness"] / 2.0).edges("|Z").fillet(5)
    for sx in (-1, 1):
        for sy in (-1, 1):
            f = f.cut(cyl(1.7, 5, sx * 24, sy * 19))
    return f


def part_link2_bracket():
    """U-bracket on joint-1 horn (link2 frame). Holds the joint-2 servo."""
    bw, bt = P["arm_bracket_width"], P["arm_bracket_thickness"]
    base = box(36, 46, 3, 0, 0, P["servo_horn_thickness"] + 1.5).edges("|Z").fillet(3)
    zb, zt = P["servo_horn_thickness"], P["j2_z"] - 5
    side_y = P["servo_depth"] / 2.0 + P["servo_horn_thickness"]
    s1 = box(bw + 4, bt, zt - zb, 0, side_y + bt / 2.0, (zb + zt) / 2.0)
    s2 = box(bw + 4, bt, zt - zb, 0, -side_y - bt / 2.0, (zb + zt) / 2.0)
    return base.union(s1).union(s2)


def _side_plates(p1, p2, lighten=True):
    bw, bt = P["arm_bracket_width"], P["arm_bracket_thickness"]
    y_in = P["servo_depth"] / 2.0 + P["servo_horn_thickness"]
    a = slot_plate(p1, p2, bw, bt, y_in, lighten)
    b = slot_plate(p1, p2, bw, bt, -y_in - bt, lighten)
    return a.union(b)


def part_link3_frame():
    return _side_plates((0, 0), (P["j3_x"], P["j3_z"]))


def part_link4_frame():
    return _side_plates((0, 0), (P["j4_x"], 0))


def part_gripper_body():
    plates = _side_plates((0, 0), (22, 0), lighten=False)
    palm = box(55, 56, 30, 42.5, 0, 0).edges("|X").fillet(4)
    rail = box(8, 66, 12, P["grip_x"] - 8, 0, 0)
    return plates.union(palm).union(rail)


def part_finger():
    """Gripper finger, frame at the prismatic joint origin. Grip pad faces -Y."""
    fl = P["finger_length"]
    f = (cq.Workplane("XZ").polyline([(-6, -11), (fl - 4, -7), (fl - 4, 7), (-6, 11)]).close()
         .extrude(4).translate((0, 4, 0)))  # y in [0,4] ... shift to [-4,4] below
    f = f.union(f.translate((0, -4, 0)))
    pad = box(fl - 18, 1.5, 12, (fl - 18) / 2.0 + 12, -4.75, 0)
    return f.union(pad)


# =============================================================================
# Robot definition: links -> list of (part_name, location in link frame)
# =============================================================================
def build_parts():
    return {
        "TB3_Waffle_Plate": (part_waffle_plate(), C["plate"]),
        "TB3_Standoff": (part_standoff(), C["standoff"]),
        "Dynamixel_XM430": (part_xm430(), C["servo"]),
        "TB3_Wheel": (part_wheel(), C["tire"]),
        "TB3_Ball_Caster": (part_caster(), C["caster"]),
        "LiPo_Battery": (part_battery(), C["battery"]),
        "OpenCR_Board": (part_opencr(), C["pcb"]),
        "Raspberry_Pi_Board": (part_rpi(), C["pcb"]),
        "LDS_Lidar": (part_lidar(), C["lidar"]),
        "Pi_Camera_Bracket": (part_camera(), C["pcb"]),
        "OMX_Base_Flange": (part_arm_flange(), C["bracket"]),
        "OMX_Link2_Bracket": (part_link2_bracket(), C["bracket"]),
        "OMX_Link3_Frame": (part_link3_frame(), C["bracket"]),
        "OMX_Link4_Frame": (part_link4_frame(), C["bracket"]),
        "OMX_Gripper_Body": (part_gripper_body(), C["gripper"]),
        "OMX_Gripper_Finger": (part_finger(), C["gripper"]),
    }


def link_layout():
    cx = P["plate_center_x"]
    servo_y = WHEEL_Y - P["wheel_width"] / 2.0 - P["servo_horn_thickness"]
    base = [
        ("TB3_Waffle_Plate", T(cx, 0, Z_BOTTOM)),
        ("TB3_Waffle_Plate", T(cx, 0, Z_MIDDLE)),
        ("TB3_Waffle_Plate", T(cx, 0, Z_TOP)),
        ("Dynamixel_XM430", T(0, servo_y, WHEEL_Z, rx=-90, rz=-90)),
        ("Dynamixel_XM430", T(0, -servo_y, WHEEL_Z, rx=90, rz=-90)),
        ("TB3_Ball_Caster", T(P["caster_x"], P["caster_y"], Z_BOTTOM)),
        ("TB3_Ball_Caster", T(P["caster_x"], -P["caster_y"], Z_BOTTOM)),
        ("LiPo_Battery", T(-80, 0, Z_BOTTOM + T_PL, rz=90)),
        ("Raspberry_Pi_Board", T(-150, 0, Z_BOTTOM + T_PL)),
        ("OpenCR_Board", T(cx, 0, Z_MIDDLE + T_PL)),
    ]
    for (x, y) in standoff_xy():
        base.append(("TB3_Standoff", T(x, y, Z_BOTTOM + T_PL)))
        base.append(("TB3_Standoff", T(x, y, Z_MIDDLE + T_PL)))
    fl = P["arm_flange_thickness"]
    return {
        "base_link": base,
        "wheel_left_link": [("TB3_Wheel", T())],
        "wheel_right_link": [("TB3_Wheel", T())],
        "base_scan": [("LDS_Lidar", T(0, 0, -30.0))],
        "camera_link": [("Pi_Camera_Bracket", T(-11, 0, -20))],
        "link1": [
            ("OMX_Base_Flange", T(0, 0, -(P["servo_depth"] - P["j1_z"]) - fl)),
            ("Dynamixel_XM430", T(P["j1_x"], 0, P["j1_z"], rz=-90)),
        ],
        "link2": [
            ("OMX_Link2_Bracket", T()),
            ("Dynamixel_XM430", T(0, 0, P["j2_z"], rx=-90, rz=180)),
        ],
        "link3": [
            ("OMX_Link3_Frame", T()),
            ("Dynamixel_XM430", T(P["j3_x"], 0, P["j3_z"], rx=-90, rz=180)),
        ],
        "link4": [
            ("OMX_Link4_Frame", T()),
            ("Dynamixel_XM430", T(P["j4_x"], 0, 0, rx=-90, rz=-90)),
        ],
        "link5": [("OMX_Gripper_Body", T())],
        "gripper_left_link": [("OMX_Gripper_Finger", T())],
        "gripper_right_link": [("OMX_Gripper_Finger", T(rx=180))],
    }


def m(v):
    """mm -> m"""
    return v / 1000.0


# Joints: name, type, parent, child, xyz(mm), rpy(rad), axis, limits, mimic
def joint_table():
    lid_h = 30.0
    return [
        ("base_joint", "fixed", "base_footprint", "base_link", (0, 0, 10.0), (0, 0, 0), None, None, None),
        ("wheel_left_joint", "continuous", "base_link", "wheel_left_link", (0, WHEEL_Y, WHEEL_Z), (-math.pi / 2, 0, 0), (0, 0, 1), None, None),
        ("wheel_right_joint", "continuous", "base_link", "wheel_right_link", (0, -WHEEL_Y, WHEEL_Z), (-math.pi / 2, 0, 0), (0, 0, 1), None, None),
        ("caster_back_left_joint", "fixed", "base_link", "caster_back_left_link", (P["caster_x"], P["caster_y"], GROUND_Z + P["caster_ball_radius"]), (0, 0, 0), None, None, None),
        ("caster_back_right_joint", "fixed", "base_link", "caster_back_right_link", (P["caster_x"], -P["caster_y"], GROUND_Z + P["caster_ball_radius"]), (0, 0, 0), None, None, None),
        ("imu_joint", "fixed", "base_link", "imu_link", (P["plate_center_x"], 0, Z_MIDDLE + T_PL + 10), (0, 0, 0), None, None, None),
        ("scan_joint", "fixed", "base_link", "base_scan", (P["lidar_x"], 0, Z_TOP_SURFACE + lid_h), (0, 0, 0), None, None, None),
        ("camera_joint", "fixed", "base_link", "camera_link", (P["camera_x"] + 11, 0, Z_TOP_SURFACE + 20), (0, 0, 0), None, None, None),
        ("arm_mount_joint", "fixed", "base_link", "link1", (P["arm_mount_x"], 0, ARM_MOUNT_Z), (0, 0, 0), None, None, None),
        ("joint1", "revolute", "link1", "link2", (P["j1_x"], 0, P["j1_z"]), (0, 0, 0), (0, 0, 1), ("j1_lower", "j1_upper"), None),
        ("joint2", "revolute", "link2", "link3", (0, 0, P["j2_z"]), (0, 0, 0), (0, 1, 0), ("j2_lower", "j2_upper"), None),
        ("joint3", "revolute", "link3", "link4", (P["j3_x"], 0, P["j3_z"]), (0, 0, 0), (0, 1, 0), ("j3_lower", "j3_upper"), None),
        ("joint4", "revolute", "link4", "link5", (P["j4_x"], 0, 0), (0, 0, 0), (0, 1, 0), ("j4_lower", "j4_upper"), None),
        ("gripper_left_joint", "prismatic", "link5", "gripper_left_link", (P["grip_x"], P["grip_y"], 0), (0, 0, 0), (0, 1, 0), ("grip_lower", "grip_upper"), None),
        ("gripper_right_joint", "prismatic", "link5", "gripper_right_link", (P["grip_x"], -P["grip_y"], 0), (0, 0, 0), (0, -1, 0), ("grip_lower", "grip_upper"), "gripper_left_joint"),
        ("end_effector_joint", "fixed", "link5", "end_effector_link", (P["ee_x"], 0, 0), (0, 0, 0), None, None, None),
    ]


LINK_MASS = {
    "base_link": "m_base", "wheel_left_link": "m_wheel", "wheel_right_link": "m_wheel",
    "base_scan": "m_lidar", "camera_link": "m_camera", "link1": "m_link1", "link2": "m_link2",
    "link3": "m_link3", "link4": "m_link4", "link5": "m_link5",
    "gripper_left_link": "m_finger", "gripper_right_link": "m_finger",
}
LINK_COLOR = {
    "base_link": ("dark_grey", (0.2, 0.2, 0.2, 1)), "wheel_left_link": ("black", (0.05, 0.05, 0.05, 1)),
    "wheel_right_link": ("black", (0.05, 0.05, 0.05, 1)), "base_scan": ("dark", (0.15, 0.15, 0.17, 1)),
    "camera_link": ("green", (0.0, 0.45, 0.2, 1)), "link1": ("aluminium", (0.75, 0.75, 0.78, 1)),
    "link2": ("aluminium", None), "link3": ("aluminium", None), "link4": ("aluminium", None),
    "link5": ("white", (0.92, 0.92, 0.92, 1)), "gripper_left_link": ("white", None), "gripper_right_link": ("white", None),
}


# =============================================================================
# Exports
# =============================================================================
def link_compound(parts, items):
    shapes = []
    for (pname, loc) in items:
        shapes.append(parts[pname][0].val().moved(loc))
    return cq.Compound.makeCompound(shapes)


def inertia(shape, mass):
    """Inertia about COM (kg m^2) from geometry with uniform density, scaled to `mass`."""
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, props)
    vol = props.Mass()  # mm^3
    com = props.CentreOfMass()
    I = props.MatrixOfInertia()  # mm^5 (density 1)
    k = mass / vol * 1e-6  # -> kg m^2
    return (m(com.X()), m(com.Y()), m(com.Z())), {
        "ixx": I.Value(1, 1) * k, "iyy": I.Value(2, 2) * k, "izz": I.Value(3, 3) * k,
        "ixy": I.Value(1, 2) * k, "ixz": I.Value(1, 3) * k, "iyz": I.Value(2, 3) * k,
    }


def export_solidworks(parts, layout):
    pdir = os.path.join(SW_DIR, "Parts")
    adir = os.path.join(SW_DIR, "Assembly")
    for d in (pdir, adir):
        shutil.rmtree(d, ignore_errors=True)
        os.makedirs(d)
    for i, (name, (wp, col)) in enumerate(parts.items(), 1):
        cq.exporters.export(wp, os.path.join(pdir, f"{i:02d}_{name}.STEP"), exportType="STEP")
        print("part", name)

    # World pose of every link at the zero joint configuration
    world = {"base_footprint": cq.Location()}
    for (jn, jt, parent, child, xyz, rpy, *_rest) in joint_table():
        world[child] = world[parent] * T(*xyz, rx=math.degrees(rpy[0]), ry=math.degrees(rpy[1]), rz=math.degrees(rpy[2]))

    arm_links = ["link1", "link2", "link3", "link4", "link5", "gripper_left_link", "gripper_right_link"]
    base_links = [l for l in layout if l not in arm_links]

    def sub_asm(name, links, ref):
        sa = cq.Assembly(name=name)
        count = {}
        for link in links:
            for (pname, loc) in layout[link]:
                count[pname] = count.get(pname, 0) + 1
                wp, col = parts[pname]
                sa.add(wp, name=f"{pname}-{count[pname]}", loc=ref.inverse * world[link] * loc, color=cq.Color(*col))
        return sa

    base_ref = cq.Location()
    arm_ref = world["link1"]
    base_asm = sub_asm("TurtleBot3_Waffle_Base", base_links, base_ref)
    arm_asm = sub_asm("OpenManipulatorX_Arm", arm_links, arm_ref)
    base_asm.export(os.path.join(adir, "TurtleBot3_Waffle_Base_Assembly.STEP"))
    arm_asm.export(os.path.join(adir, "OpenManipulatorX_Arm_Assembly.STEP"))
    full = cq.Assembly(name="TB3_OpenManipulatorX_Mobile_Manipulator")
    full.add(base_asm, name="TurtleBot3_Waffle_Base")
    full.add(arm_asm, name="OpenManipulatorX_Arm", loc=arm_ref)
    full.export(os.path.join(adir, "TB3_OpenManipulatorX_Full_Assembly.STEP"))
    print("assemblies done")


def export_parameters():
    d = os.path.join(SW_DIR, "Parameters")
    os.makedirs(d, exist_ok=True)
    derived = {"bottom_plate_z": Z_BOTTOM, "middle_plate_z": Z_MIDDLE, "top_plate_z": Z_TOP,
               "caster_height": CASTER_H, "arm_mount_z": ARM_MOUNT_Z, "wheel_center_z": WHEEL_Z}
    with open(os.path.join(d, "parameters.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["parameter", "value", "unit"])
        for k, v in list(P.items()) + list(derived.items()):
            unit = "rad" if k.endswith(("_lower", "_upper")) and not k.startswith("grip") else (
                "m" if k.startswith("grip_l") or k.startswith("grip_u") else
                "kg" if k.startswith("m_") else "-" if k.endswith(("_nx", "_ny", "effort", "velocity")) else "mm")
            w.writerow([k, round(v, 6) if isinstance(v, float) else v, unit])
    # SolidWorks Tools > Equations > Import  (Global Variables)
    with open(os.path.join(d, "SolidWorks_Global_Variables.txt"), "w") as f:
        for k, v in list(P.items()) + list(derived.items()):
            if k.startswith("m_") or k.endswith(("_lower", "_upper", "effort", "velocity")):
                continue
            if k.endswith(("_nx", "_ny")):
                f.write(f'"{k}" = {v}\n')
            else:
                f.write(f'"{k}" = {v:.4f}mm\n')


def fnum(v):
    return f"{v:.6g}" if abs(v) >= 1e-12 else "0"


def export_urdf(parts, layout):
    mesh_dir = os.path.join(PKG_DIR, "meshes")
    shutil.rmtree(mesh_dir, ignore_errors=True)
    os.makedirs(mesh_dir)
    inert = {}
    for link, items in layout.items():
        comp = link_compound(parts, items)
        cq.exporters.export(cq.Workplane().add(comp), os.path.join(mesh_dir, f"{link}.stl"),
                            tolerance=0.15, angularTolerance=0.25)
        inert[link] = inertia(comp, P[LINK_MASS[link]])
        print("mesh", link)

    xacro_props = {
        "wheel_radius": m(P["wheel_radius"]), "wheel_width": m(P["wheel_width"]),
        "wheel_separation": m(P["wheel_separation"]),
    }

    def xyz_s(v):
        return " ".join(fnum(m(a)) for a in v)

    def rpy_s(v):
        return " ".join(fnum(a) for a in v)

    def build(xacro):
        L = []
        a = L.append
        a('<?xml version="1.0"?>')
        a("<!-- TurtleBot3 Waffle Pi + OpenMANIPULATOR-X  |  generated by source/generate.py -->")
        if xacro:
            a('<robot name="tb3_omx" xmlns:xacro="http://www.ros.org/wiki/xacro">')
            a('  <xacro:arg name="use_gazebo" default="false"/>')
            a("  <!-- ===== Parameters ===== -->")
            for k in ("j1_lower", "j1_upper", "j2_lower", "j2_upper", "j3_lower", "j3_upper", "j4_lower", "j4_upper", "grip_lower", "grip_upper", "arm_effort", "arm_velocity"):
                a(f'  <xacro:property name="{k}" value="{fnum(P[k])}"/>')
            for k, v in xacro_props.items():
                a(f'  <xacro:property name="{k}" value="{fnum(v)}"/>')
            a('  <xacro:property name="mesh_path" value="package://' + PKG + '/meshes"/>')
        else:
            a('<robot name="tb3_omx">')
        mesh = (lambda n: "${mesh_path}/" + n + ".stl") if xacro else (lambda n: f"package://{PKG}/meshes/{n}.stl")
        lim = (lambda k: "${" + k + "}") if xacro else (lambda k: fnum(P[k]))

        done_mat = set()
        for name, (mname, rgba) in LINK_COLOR.items():
            if rgba and mname not in done_mat:
                done_mat.add(mname)
                a(f'  <material name="{mname}"><color rgba="{" ".join(str(c) for c in rgba)}"/></material>')

        a('  <link name="base_footprint"/>')
        for link in ["imu_link", "end_effector_link"]:
            a(f'  <link name="{link}"/>')
        for c in ("caster_back_left_link", "caster_back_right_link"):
            a(f'  <link name="{c}">')
            a(f'    <collision><geometry><sphere radius="{fnum(m(P["caster_ball_radius"]))}"/></geometry></collision>')
            a(f'    <inertial><mass value="{P["m_caster"]}"/><inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/></inertial>')
            a("  </link>")

        for link in layout:
            com, I = inert[link]
            a(f'  <link name="{link}">')
            a("    <visual>")
            a('      <origin xyz="0 0 0" rpy="0 0 0"/>')
            a(f'      <geometry><mesh filename="{mesh(link)}" scale="0.001 0.001 0.001"/></geometry>')
            a(f'      <material name="{LINK_COLOR[link][0]}"/>')
            a("    </visual>")
            a("    <collision>")
            if link == "base_link":
                zc = (Z_BOTTOM - 8 + Z_TOP_SURFACE) / 2.0
                a(f'      <origin xyz="{xyz_s((P["plate_center_x"], 0, zc))}" rpy="0 0 0"/>')
                a(f'      <geometry><box size="{xyz_s((P["plate_length"], P["plate_width"], Z_TOP_SURFACE - Z_BOTTOM + 8))}"/></geometry>')
            elif link.startswith("wheel"):
                a('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                if xacro:
                    a('      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>')
                else:
                    a(f'      <geometry><cylinder radius="{fnum(m(P["wheel_radius"]))}" length="{fnum(m(P["wheel_width"]))}"/></geometry>')
            else:
                a('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                a(f'      <geometry><mesh filename="{mesh(link)}" scale="0.001 0.001 0.001"/></geometry>')
            a("    </collision>")
            a("    <inertial>")
            a(f'      <origin xyz="{" ".join(fnum(c) for c in com)}" rpy="0 0 0"/>')
            a(f'      <mass value="{fnum(P[LINK_MASS[link]])}"/>')
            a('      <inertia ' + " ".join(f'{k}="{I[k]:.6e}"' for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")) + "/>")
            a("    </inertial>")
            a("  </link>")

        for (jn, jt, parent, child, xyz, rpy, axis, limits, mimic) in joint_table():
            a(f'  <joint name="{jn}" type="{jt}">')
            a(f'    <parent link="{parent}"/>')
            a(f'    <child link="{child}"/>')
            if xacro and jn.startswith("wheel"):
                sign = "" if "left" in jn else "-"
                a(f'    <origin xyz="0 {sign}${{wheel_separation/2}} {fnum(m(xyz[2]))}" rpy="{rpy_s(rpy)}"/>')
            else:
                a(f'    <origin xyz="{xyz_s(xyz)}" rpy="{rpy_s(rpy)}"/>')
            if axis:
                a(f'    <axis xyz="{" ".join(str(v) for v in axis)}"/>')
            if limits:
                eff = lim("arm_effort")
                vel = lim("arm_velocity") if jt == "revolute" else "0.1"
                a(f'    <limit lower="{lim(limits[0])}" upper="{lim(limits[1])}" effort="{eff}" velocity="{vel}"/>')
                a('    <dynamics damping="0.1" friction="0.0"/>')
            if jt == "continuous":
                a('    <dynamics damping="0.01" friction="0.0"/>')
            if mimic:
                a(f'    <mimic joint="{mimic}" multiplier="1" offset="0"/>')
            a("  </joint>")

        if xacro:
            a('  <xacro:if value="$(arg use_gazebo)">')
            a(f'    <xacro:include filename="$(find {PKG})/urdf/tb3_omx.gazebo.xacro"/>')
            a("  </xacro:if>")
        a("</robot>")
        return "\n".join(L) + "\n"

    udir = os.path.join(PKG_DIR, "urdf")
    os.makedirs(udir, exist_ok=True)
    with open(os.path.join(udir, "tb3_omx.urdf.xacro"), "w") as f:
        f.write(build(True))
    with open(os.path.join(udir, "tb3_omx.urdf"), "w") as f:
        f.write(build(False))
    print("urdf done")


def main():
    parts = build_parts()
    layout = link_layout()
    export_solidworks(parts, layout)
    export_parameters()
    export_urdf(parts, layout)


if __name__ == "__main__":
    main()
