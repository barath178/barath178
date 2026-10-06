"""Rebuild the complete myoelectric hand around the geometry in 'myoelectric hand buy.step'.

World frame (same as the original STEP, units mm):
  X  across the palm, Z  along the arm (+Z towards the forearm / wrist puck, fingers hang to -Z),
  Y  across the hand thickness. Hand is symmetric about the plane y = -23 (wrist puck axis).
"""
import sys, os, math, json
import cadquery as cq
from OCP.STEPControl import STEPControl_Reader

SRC, OUT = sys.argv[1], sys.argv[2]
os.makedirs(os.path.join(OUT, "parts_print"), exist_ok=True)
os.makedirs(os.path.join(OUT, "parts_hardware_reference"), exist_ok=True)

YC = -23.0                      # symmetry plane / wrist axis
CLR = 0.2                       # extra radial clearance on NEW printed holes

# ---------------------------------------------------------------- helpers
def xz(y_top):
    """Workplane whose local (u, v) = world (x, z); extrude(d) goes from y_top to y_top - d."""
    return cq.Workplane(cq.Plane(origin=(0, y_top, 0), xDir=(1, 0, 0), normal=(0, -1, 0)))

def slab(y0, y1):
    return xz(y1), (y1 - y0)

def cyl_y(x, z, r, y0, y1):
    return xz(y1).center(x, z).circle(r).extrude(y1 - y0)

def cyl_x(y, z, r, x0, x1):
    return cq.Workplane("YZ", origin=(x0, 0, 0)).center(y, z).circle(r).extrude(x1 - x0)

def cyl_z(x, y, r, z0, z1):
    return cq.Workplane("XY", origin=(0, 0, z0)).center(x, y).circle(r).extrude(z1 - z0)

def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))

def mirror_y(wp):
    return wp.mirror("XZ", basePointVector=(0, YC, 0))

def hex_nut_y(x, z, y0, y1, af):          # hex nut, axis along Y, across-flats af
    return xz(y1).center(x, z).polygon(6, af / math.cos(math.pi / 6)).extrude(y1 - y0)

# ------------------------------------------------- original geometry (exact)
rd = STEPControl_Reader(); rd.ReadFile(SRC); rd.TransferRoots()
shells = cq.Shape.cast(rd.Shape(1)).Shells()

def main_face(shell):
    fs = [f for f in shell.Faces() if f.geomType() == "PLANE" and abs(f.normalAt().y) > 0.99]
    return max(fs, key=lambda f: f.Area())

def solid_from_face(face, y_top, thick, grow):
    """Extrude an original planar face. grow = edge-fillet inset to add back to the outline."""
    outer = face.outerWire()
    if grow:
        outer = outer.offset2D(grow, "intersection")[0]   # same outline as "arc" here, no sliver edges
    f = cq.Face.makeFromWires(outer, face.innerWires())
    f = f.translate(cq.Vector(0, y_top - f.Center().y, 0))
    return cq.Workplane().add(cq.Solid.extrudeLinear(f, cq.Vector(0, -thick, 0)))

# Finger (Component75): face at y=-1.5 is inset 0.5 by the edge fillet; true plate 3.0 thick (-4.5..-1.5)
finger_proto = solid_from_face(main_face(shells[0]), -1.5, 3.0, 0.5)
# Thumb (Component77): same construction, same 3.0 sheet
thumb_proto = solid_from_face(main_face(shells[2]), -3.65, 3.0, 0.5)
# Side plate (Component78): face y=3 -> plate y 3..6 (3 mm)
plate_face = main_face(shells[3])
side_plate_f = solid_from_face(plate_face, 6.0, 3.0, 0)
# Front tab (Component76): face y=0 -> tab y 0..3
tab_f = solid_from_face(main_face(shells[1]), 3.0, 3.0, 0)

# ------------------------------------------------- layout constants (from the original holes)
PIV_F = (-17.64, -17.15)   # finger pivot (Ø8 finger hole, Ø2.9 plate hole)
PIN_F = (-9.87, -27.70)    # Ø3 finger hole -> finger coupling rod
PIV_T = (23.87, -11.50)    # thumb pivot (Ø8 thumb hole, Ø2.9 plate hole)
PIN_T = (19.97, 0.97)      # Ø8 thumb hole -> thumb/rocker locking pin
CRANK = (0.0, 0.0)         # Ø5.2 plate hole -> drive shaft
AUX = (0.0, 15.6)          # Ø5.2 plate hole -> motor wiring pass-through
TAB_SCREWS = [(-7.0, 19.9), (11.0, 19.9)]  # Ø2.7 tab / Ø2.9 plate holes -> M2.5
REL_PIN = (13.0, -3.0)     # NEW: emergency-release pin through rocker + plate

FY = [-1.5 - i * (40.0 / 3.0) for i in range(4)]   # finger top faces: 4 fingers, symmetric about y=-23
# finger i occupies y FY[i]-3 .. FY[i]   -> -4.5..-1.5, -17.83..-14.83, -31.17..-28.17, -44.5..-41.5

parts = []   # (name, workplane, color, kind)  kind: 'print' or 'hw'
def add(name, wp, color, kind="print"):
    parts.append((name, wp, color, kind))

# --- side plates
side_plate = side_plate_f.cut(cyl_y(*REL_PIN, 1.6 + 0.0, 2.0, 7.0))
add("side_plate_front", side_plate, (0.85, 0.05, 0.05))
add("side_plate_back", mirror_y(side_plate), (0.85, 0.05, 0.05))

# --- fingers
for i, yt in enumerate(FY):
    add(f"finger_{i+1}", finger_proto.translate((0, yt + 1.5, 0)), (0.93, 0.83, 0.55))

# --- thumb (outside the front plate, as in the screenshots) and orange rocker
THUMB_Y = (6.5, 9.5)
ROCK_Y = (9.5, 12.5)
thumb = thumb_proto.translate((0, THUMB_Y[1] + 3.65, 0))
add("thumb", thumb, (0.93, 0.83, 0.55))

def hull_pts(circles, n=48):
    pts = []
    for (cx, cz, r) in circles:
        for k in range(n):
            a = 2 * math.pi * k / n
            pts.append((cx + r * math.cos(a), cz + r * math.sin(a)))
    pts = sorted(set(pts))
    def cross(o, a, b): return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0: lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0: up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]

rocker_outline = hull_pts([(*PIV_T, 7.75), (*PIN_T, 7.0), (*REL_PIN, 4.5)])
rocker = (xz(ROCK_Y[1]).polyline(rocker_outline).close().extrude(3.0)
          .cut(cyl_y(*PIV_T, 1.6 + CLR, 0, 20))          # M3 pivot
          .cut(cyl_y(*PIN_T, 4.0, 0, 20))                 # Ø8 thumb pin (matches thumb hole)
          .cut(cyl_y(*REL_PIN, 1.6 + CLR, 0, 20)))        # Ø3 release pin
add("thumb_rocker", rocker, (1.0, 0.55, 0.05))

# --- N20 gear motor, coaxial with the crank (plate hole at 0,0); output shaft D3, tip at y=2.8
N20 = dict(w=12.0, h=10.0, face_y=-9.2, gear_len=9.0, motor_len=15.0, cap_len=1.8)
gb0 = N20["face_y"] - N20["gear_len"]; mo0 = gb0 - N20["motor_len"]; cap0 = mo0 - N20["cap_len"]
motor = (box(-6.0, 6.0, gb0, N20["face_y"], -5.0, 5.0)
         .union(cyl_y(0, 0, 6.0, mo0, gb0).intersect(box(-6, 6, mo0, gb0, -5.0, 5.0)))
         .union(box(-5.0, 5.0, cap0, mo0, -4.0, 4.0))
         .union(cyl_y(0, 0, 2.0, N20["face_y"], N20["face_y"] + 1.0))                    # bearing boss Ø4
         .union(cyl_y(0, 0, 1.5, N20["face_y"], N20["face_y"] + 10.0).cut(box(-2, 2, -10, 10, 1.0, 2.0)))  # D-shaft Ø3x10, flat 2.5
         .cut(cyl_y(-4.5, 0, 0.8, gb0 + 4, N20["face_y"] + 0.1)).cut(cyl_y(4.5, 0, 0.8, gb0 + 4, N20["face_y"] + 0.1)))
add("n20_gearmotor_6V_ref", motor, (0.25, 0.75, 0.4), "hw")

# --- base bracket = both original tabs + base plate + motor housing (one printed part)
BASE_Z = (24.0, 30.0)
base_screws = [(15.0, -15.0), (15.0, -31.0), (-6.0, -39.0), (6.0, -39.0)]      # M3 into wrist puck
HOUS = dict(x0=-12.0, x1=12.0, y0=-35.0, y1=-7.2, z0=-7.0)
base = (box(-14.0, 19.0, -49.0, 3.0, *BASE_Z)
        .union(tab_f).union(mirror_y(tab_f))
        .union(box(HOUS["x0"], HOUS["x1"], HOUS["y0"], HOUS["y1"], HOUS["z0"], BASE_Z[0] + 0.5)))
base = (base.cut(box(-6.0 - 0.15, 6.0 + 0.15, HOUS["y0"] - 1, N20["face_y"], -5.0 - 0.15, 5.0 + 0.15))   # motor pocket
            .cut(cyl_y(0, 0, 2.0 + CLR, -12, 0))                                                  # boss / shaft hole
            .cut(cyl_y(-4.5, 0, 0.8 + 0.1, -12, 0)).cut(cyl_y(4.5, 0, 0.8 + 0.1, -12, 0))        # M1.6 motor screws
            .cut(cyl_z(0, YC, 9.1, 0, 40)))                                                      # cable hole (= puck bore)
for (x, y) in base_screws:
    base = base.cut(cyl_z(x, y, 1.6 + CLR, 20, 40)).cut(cyl_z(x, y, 3.0 + CLR, 20, BASE_Z[0] + 3.0))
add("base_bracket_with_motor_housing", base, (0.85, 0.72, 0.5))

# --- wrist puck (Component79): exact from original surfaces
puck = (cyl_z(0, YC, 25.0, 32.6, 47.3).edges("<Z").fillet(1.5)
        .union(cyl_z(0, YC, 22.0, 30.0, 32.6))
        .cut(cyl_z(0, YC, 9.1, 20, 60))
        .cut(cyl_z(7.55, -12.2, 0.75, 20, 60)))
for (x, y) in base_screws:                                    # M3 pilot Ø2.5 (self-tap) from below
    puck = puck.cut(cyl_z(x, y, 1.25, 29, 40))
SOCK_SCREWS = [(14.5 * math.cos(math.radians(a)), YC + 14.5 * math.sin(math.radians(a))) for a in (30, 150, 270)]
for (x, y) in SOCK_SCREWS:                                    # tapped M3 from above
    puck = puck.cut(cyl_z(x, y, 1.25, 39, 48))
add("wrist_puck", puck, (0.85, 0.05, 0.05))

# --- finger pivot tube (Ø8 x Ø2.5 bore for M3 screws, spans between plates)
tube = cyl_y(*PIV_F, 4.0 - 0.15, -49.0, 3.0).cut(cyl_y(*PIV_F, 1.25, -60, 10))
add("finger_pivot_tube", tube, (0.1, 0.1, 0.1))
# spacer sleeves on the pivot tube between fingers
for i in range(3):
    y1, y0 = FY[i] - 3.0, FY[i + 1]
    add(f"pivot_spacer_{i+1}", cyl_y(*PIV_F, 6.0, y0 + 0.2, y1 - 0.2).cut(cyl_y(*PIV_F, 4.0 + CLR, -60, 10)), (0.1, 0.1, 0.1))
add("pivot_spacer_front", cyl_y(*PIV_F, 5.0, -1.4, 0.85).cut(cyl_y(*PIV_F, 4.0 + CLR, -60, 10)), (0.1, 0.1, 0.1))   # link passes the bare tube above this
add("pivot_spacer_back", mirror_y(cyl_y(*PIV_F, 6.0, -1.3, 2.8).cut(cyl_y(*PIV_F, 4.0 + CLR, -60, 10))), (0.1, 0.1, 0.1))

# finger coupling rod (Ø3 steel) + spacer sleeves between fingers
add("finger_coupling_rod_M3_threaded_x52", cyl_y(*PIN_F, 1.45, -46.9, 5.2), (0.6, 0.6, 0.65), "hw")
for i in range(3):
    y1, y0 = FY[i] - 3.0, FY[i + 1]
    add(f"rod_spacer_{i+1}", cyl_y(*PIN_F, 3.0, y0 + 0.2, y1 - 0.2).cut(cyl_y(*PIN_F, 1.5 + CLR, -60, 10)), (0.1, 0.1, 0.1))
add("nut_M3_rod_front", hex_nut_y(*PIN_F, 2.8, 5.2, 5.5).cut(cyl_y(*PIN_F, 1.5, -60, 10)), (0.6, 0.6, 0.65), "hw")
add("nut_M3_rod_back", hex_nut_y(*PIN_F, -46.9, -44.5 - 0.0, 5.5).cut(cyl_y(*PIN_F, 1.5, -60, 10)), (0.6, 0.6, 0.65), "hw")

# --- drive: crank on the N20 shaft + dog-leg link to the finger coupling rod.
# Four-bar: ground 24.60 (shaft->pivot), crank 5.0, coupler 34.40 (pin-pin), rocker 13.10.
# Crank at 72 deg puts the linkage at dead-centre in the modelled (closed) pose -> grip self-locks;
# one crank turn swings the fingers 0 .. 91 deg open. Bend keeps >=1.1 mm off the pivot tube and nuts.
CR, CANG = 5.0, math.radians(72.0)
CTIP = (CR * math.cos(CANG), CR * math.sin(CANG))
_dx, _dz = PIN_F[0] - CTIP[0], PIN_F[1] - CTIP[1]; _n = math.hypot(_dx, _dz)
LINK_WP = (CTIP[0] + 0.7 * _dx - 12.0 * _dz / _n, CTIP[1] + 0.7 * _dz + 12.0 * _dx / _n)
CRANK_Y = (-0.85, 0.85); LINK_Y = (1.0, 2.8)   # shaft ends inside the crank; link runs against the side plate
crank = (xz(CRANK_Y[1]).polyline(hull_pts([(0, 0, 5.0), (*CTIP, 3.5)])).close().extrude(CRANK_Y[1] - CRANK_Y[0])
         .cut(cyl_y(0, 0, 1.5 + 0.1, -10, 10).cut(box(-2, 2, -10, 10, 1.0 + 0.1, 2.0)))   # D bore for the N20 shaft
         .cut(cyl_y(*CTIP, 1.5 - 0.05, -10, 10)))                                          # Ø3 pin, press fit
add("crank_arm", crank, (0.1, 0.1, 0.1))
link = (xz(LINK_Y[1]).polyline(hull_pts([(*CTIP, 3.5), (*LINK_WP, 3.5)])).close().extrude(LINK_Y[1] - LINK_Y[0])
        .union(xz(LINK_Y[1]).polyline(hull_pts([(*LINK_WP, 3.5), (*PIN_F, 3.5)])).close().extrude(LINK_Y[1] - LINK_Y[0]))
        .cut(cyl_y(*CTIP, 1.5 + CLR, -10, 10)).cut(cyl_y(*PIN_F, 1.5 + CLR, -10, 10)))
add("drive_link", link, (0.1, 0.1, 0.1))
add("crank_pin_D3x4", cyl_y(*CTIP, 1.45, CRANK_Y[0], LINK_Y[1]), (0.6, 0.6, 0.65), "hw")

# --- fasteners (reference geometry)
def screw_y(name, x, z, y_head_top, y_tip, d=3.0, head_d=5.5, head_h=3.0):
    """screw along Y, head from y_head_top going +, shank to y_tip."""
    s = cyl_y(x, z, d / 2 * 0.95, min(y_tip, y_head_top), max(y_tip, y_head_top))
    if y_head_top > y_tip:
        h = cyl_y(x, z, head_d / 2, y_head_top, y_head_top + head_h)
    else:
        h = cyl_y(x, z, head_d / 2, y_head_top - head_h, y_head_top)
    add(name, s.union(h), (0.2, 0.8, 0.3), "hw")

screw_y("screw_M3x8_pivot_front", *PIV_F, 6.0, -2.0)
add("screw_M3x8_pivot_back", mirror_y(cyl_y(*PIV_F, 1.42, -2.0, 6.0).union(cyl_y(*PIV_F, 2.75, 6.0, 9.0))), (0.2, 0.8, 0.3), "hw")
screw_y("screw_M3x14_thumb_pivot", *PIV_T, 12.5, -1.5)
add("nut_M3_thumb_pivot", hex_nut_y(*PIV_T, 0.6, 3.0, 5.5).cut(cyl_y(*PIV_T, 1.5, -10, 10)), (0.6, 0.6, 0.65), "hw")
add("thumb_pin_D8x6", cyl_y(*PIN_T, 3.95, 6.5, 12.5).union(cyl_y(*PIN_T, 5.0, 12.5, 13.5)), (0.2, 0.8, 0.3), "hw")
add("release_pin_D3x12", cyl_y(*REL_PIN, 1.45, 3.0, 12.5).union(cyl_y(*REL_PIN, 3.0, 12.5, 14.5)), (0.2, 0.8, 0.3), "hw")
for k, (x, z) in enumerate(TAB_SCREWS):
    s = cyl_y(x, z, 1.2, -2.0, 6.0).union(cyl_y(x, z, 2.25, 6.0, 8.5))
    add(f"screw_M2.5x8_tab_front_{k+1}", s, (0.2, 0.8, 0.3), "hw")
    add(f"screw_M2.5x8_tab_back_{k+1}", mirror_y(s), (0.2, 0.8, 0.3), "hw")
    add(f"nut_M2.5_tab_front_{k+1}", hex_nut_y(x, z, -2.0, 0.0, 5.0).cut(cyl_y(x, z, 1.25, -10, 10)), (0.6, 0.6, 0.65), "hw")
    add(f"nut_M2.5_tab_back_{k+1}", mirror_y(hex_nut_y(x, z, -2.0, 0.0, 5.0).cut(cyl_y(x, z, 1.25, -10, 10))), (0.6, 0.6, 0.65), "hw")
for k, (x, y) in enumerate(base_screws):     # head in base counterbore, 5 mm into the puck
    add(f"screw_M3x8_base_to_puck_{k+1}", cyl_z(x, y, 1.42, 27.0, 35.0).union(cyl_z(x, y, 2.75, 24.0, 27.0)), (0.2, 0.8, 0.3), "hw")
for k, x in enumerate((-4.5, 4.5)):           # N20 gearbox face screws through the housing front wall
    add(f"screw_M1.6x4_motor_{k+1}", cyl_y(x, 0, 0.78, -11.2, -7.2).union(cyl_y(x, 0, 1.5, -7.2, -6.7)), (0.2, 0.8, 0.3), "hw")

# ------------------------------------------------- forearm socket + electronics
R_SOCK, Z0, Z_CYL, Z_TOP, R_TOP, WALL = 27.5, 47.3, 112.0, 210.0, 42.0, 3.0
def socket_solid(grow=0.0):
    pts = [(0, Z0), (R_SOCK + grow, Z0), (R_SOCK + grow, Z_CYL), (R_TOP + grow, Z_TOP), (0, Z_TOP)]
    return (cq.Workplane("XZ").polyline(pts).close().revolve(360, (0, 0, 0), (0, 1, 0))
            .translate((0, YC, 0)))
outer = socket_solid()
inner_pts = [(0, Z0 + 6.0), (R_SOCK - WALL, Z0 + 6.0), (R_SOCK - WALL, Z_CYL), (R_TOP - WALL, Z_TOP), (R_TOP - WALL, Z_TOP + 1), (0, Z_TOP + 1)]
inner = cq.Workplane("XZ").polyline(inner_pts).close().revolve(360, (0, 0, 0), (0, 1, 0)).translate((0, YC, 0))
emg_boss = box(21.0, 45.0, YC - 21.0, YC + 21.0, 130.0, 172.0).intersect(outer)
socket = outer.cut(inner).union(emg_boss).cut(box(21.0, 24.2, YC - 19.3, YC + 19.3, 132.5, 169.5))
socket = socket.cut(cyl_z(0, YC, 9.1, 40, 60))
for (x, y) in SOCK_SCREWS:
    socket = socket.cut(cyl_z(x, y, 1.6 + CLR, 40, 60)).cut(cyl_z(x, y, 3.0 + CLR, Z0 + 2.8, 60))
socket = socket.cut(cyl_x(YC, 100.0, 4.0, 15, 40))                  # cable hole to electronics box
socket = socket.cut(cyl_x(YC, 151.0, 4.0, 15, 40))                  # EMG cable hole
add("forearm_socket", socket, (0.9, 0.9, 0.92))

# EMG sensor (MyoWare 2.0 envelope) inside the socket pocket, electrodes facing the skin (-X)
emg = box(22.4, 24.0, YC - 18.8, YC + 18.8, 133.0, 169.0)
for dz in (-12, 0, 12):
    emg = emg.union(cyl_x(YC, 151.0 + dz, 3.5, 18.4, 22.4))
add("emg_sensor_myoware2_ref", emg, (0.1, 0.3, 0.8), "hw")

# electronics enclosure, conforming to the socket on the +X side
EB = dict(x0=20.0, x1=52.0, y0=YC - 22.0, y1=YC + 22.0, z0=50.0, z1=120.0)
enc = box(EB["x0"], EB["x1"], EB["y0"], EB["y1"], EB["z0"], EB["z1"]).cut(outer)
cavity = box(EB["x0"], EB["x1"] + 1, EB["y0"] + 2, EB["y1"] - 2, EB["z0"] + 2, EB["z1"] - 2).cut(socket_solid(2.0))
enc = enc.cut(cavity)
bosses = [(YC - 17.0, 56.0), (YC + 17.0, 56.0), (YC - 17.0, 114.0), (YC + 17.0, 114.0)]
for (y, z) in bosses:
    enc = enc.union(cyl_x(y, z, 3.5, 42.0, EB["x1"]).cut(cyl_x(y, z, 1.1, 40, 60)))
enc = enc.cut(cyl_x(YC, 100.0, 4.0, 15, 40))
add("electronics_enclosure", enc, (0.3, 0.3, 0.32))
lid = box(EB["x1"], EB["x1"] + 2.0, EB["y0"], EB["y1"], EB["z0"], EB["z1"])
for (y, z) in bosses:
    lid = lid.cut(cyl_x(y, z, 1.3 + CLR, 40, 60))
lid = lid.cut(box(EB["x1"] - 1, EB["x1"] + 3, YC - 4, YC + 4, 70, 82))   # power switch window
add("enclosure_lid", lid, (0.3, 0.3, 0.32))
for k, (x, y) in enumerate(SOCK_SCREWS):     # socket floor -> puck
    add(f"screw_M3x10_socket_{k+1}", cyl_z(x, y, 1.42, Z0 + 2.8 - 10.0, Z0 + 2.8).union(cyl_z(x, y, 2.75, Z0 + 2.8, Z0 + 5.8)), (0.2, 0.8, 0.3), "hw")
for k, (y, z) in enumerate(bosses):
    add(f"screw_M2.5x8_lid_{k+1}", cyl_x(y, z, 1.2, 46.0, EB["x1"] + 2.0).union(cyl_x(y, z, 2.25, EB["x1"] + 2.0, EB["x1"] + 4.0)), (0.2, 0.8, 0.3), "hw")
add("battery_2S_lipo_50x30x10_ref", box(31.0, 41.0, YC - 15.0, YC + 15.0, 58.0, 108.0), (0.2, 0.2, 0.7), "hw")
add("arduino_nano_ref", box(43.0, 49.0, YC + 0.5, YC + 19.0, 63.0, 106.2), (0.1, 0.4, 0.8), "hw")
add("motor_driver_drv8833_ref", box(43.0, 47.0, YC - 18.5, YC - 0.5, 63.0, 79.0), (0.6, 0.1, 0.6), "hw")

# ------------------------------------------------- export
assy = cq.Assembly(name="myoelectric_hand_complete")
manifest = []
for name, wp, col, kind in parts:
    shape = wp.val() if len(wp.vals()) == 1 else cq.Compound.makeCompound(wp.vals())
    assy.add(shape, name=name, color=cq.Color(*col))
    sub = "parts_print" if kind == "print" else "parts_hardware_reference"
    cq.exporters.export(cq.Workplane().add(shape), os.path.join(OUT, sub, name + ".step"))
    if kind == "print":
        cq.exporters.export(cq.Workplane().add(shape), os.path.join(OUT, sub, name + ".stl"), tolerance=0.01, angularTolerance=0.1)
    bb = shape.BoundingBox()
    manifest.append(dict(name=name, kind=kind, valid=shape.isValid(), volume_mm3=round(shape.Volume(), 1),
                         n_solids=len(shape.Solids()),
                         bbox=[round(v, 2) for v in (bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax)],
                         size=[round(bb.xlen, 2), round(bb.ylen, 2), round(bb.zlen, 2)]))
assy.save(os.path.join(OUT, "myoelectric_hand_complete.step"))
json.dump(manifest, open(os.path.join(OUT, "manifest.json"), "w"), indent=1)
print("parts:", len(parts))
