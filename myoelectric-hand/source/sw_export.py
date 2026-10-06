"""Make a SolidWorks-friendly package from the generated parts.

- groups parts that are identical up to a translation (e.g. the 4 fingers) into ONE part file
- writes  solidworks/parts/<Part Name>.step   (one per unique part, mm)
- writes  solidworks/Myoelectric Hand Assembly.step  (true instanced assembly: each part stored once)
- writes  solidworks/instances.csv  and the macro  solidworks/BuildSolidWorksAssembly.bas
usage: python sw_export.py <myoelectric-hand dir>
"""
import sys, os, json, re, csv
import cadquery as cq
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorType
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Trsf, gp_Vec
from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut

ROOT = sys.argv[1]
SW = os.path.join(ROOT, "solidworks"); os.makedirs(os.path.join(SW, "parts"), exist_ok=True)
manifest = json.load(open(os.path.join(ROOT, "cad", "manifest.json")))

NICE = {  # prototype key -> SolidWorks part name
    "side_plate": "Side Plate", "finger": "Finger", "thumb": "Thumb", "thumb_rocker": "Thumb Rocker",
    "base_bracket_with_motor_housing": "Base Bracket with Motor Housing", "wrist_puck": "Wrist Puck",
    "finger_pivot_tube": "Finger Pivot Tube", "pivot_spacer": "Pivot Spacer 9.9",
    "pivot_spacer_front": "Pivot Spacer Front", "pivot_spacer_back": "Pivot Spacer Back",
    "rod_spacer": "Rod Spacer", "crank_arm": "Crank Arm", "drive_link": "Drive Link",
    "forearm_socket": "Forearm Socket", "electronics_enclosure": "Electronics Enclosure", "enclosure_lid": "Enclosure Lid",
    "n20_gearmotor_6V_ref": "HW N20 Gearmotor 6V", "finger_coupling_rod_M3_threaded_x52": "HW M3 Threaded Rod 52",
    "nut_M3_rod": "HW M3 Nut", "nut_M3_thumb_pivot": "HW M3 Nut", "crank_pin_D3x4": "HW Pin D3x3.6",
    "screw_M3x8_pivot": "HW M3x8 Socket Screw", "screw_M3x14_thumb_pivot": "HW M3x14 Socket Screw",
    "thumb_pin_D8x6": "HW Thumb Pin D8x6", "release_pin_D3x12": "HW Release Pin D3x12",
    "screw_M2.5x8_tab": "HW M2.5x8 Screw", "nut_M2.5_tab": "HW M2.5 Nut", "screw_M3x8_base_to_puck": "HW M3x8 Socket Screw",
    "screw_M1.6x4_motor": "HW M1.6x4 Screw", "screw_M3x10_socket": "HW M3x10 Socket Screw",
    "screw_M2.5x8_lid": "HW M2.5x8 Screw", "emg_sensor_myoware2_ref": "HW MyoWare 2.0 EMG Sensor",
    "battery_2S_lipo_50x30x10_ref": "HW Battery 2S LiPo 50x30x10", "arduino_nano_ref": "HW Arduino Nano",
    "motor_driver_drv8833_ref": "HW DRV8833 Motor Driver",
}
COLOR = [("side_plate", (0.85, 0.08, 0.08)), ("wrist_puck", (0.85, 0.08, 0.08)), ("thumb_rocker", (1.0, 0.55, 0.05)),
         ("finger_pivot", (0.12, 0.12, 0.12)), ("finger", (0.93, 0.83, 0.55)), ("thumb", (0.93, 0.83, 0.55)),
         ("base", (0.85, 0.72, 0.5)), ("n20", (0.3, 0.75, 0.4)), ("socket", (0.88, 0.88, 0.9)),
         ("enclosure", (0.35, 0.35, 0.38)), ("lid", (0.35, 0.35, 0.38)), ("screw", (0.2, 0.8, 0.3)),
         ("pin", (0.2, 0.8, 0.3)), ("nut", (0.6, 0.6, 0.65)), ("rod", (0.6, 0.6, 0.65)),
         ("emg", (0.1, 0.3, 0.8)), ("battery", (0.2, 0.2, 0.7)), ("arduino", (0.1, 0.4, 0.8)), ("driver", (0.6, 0.1, 0.6))]

def key_of(name):
    k = re.sub(r"_\d+$", "", name)
    k = re.sub(r"_(front|back)$", "", k) if not k.startswith("pivot_spacer_") else k
    k = re.sub(r"_(front|back)$", "", k) if k.startswith(("screw_M2.5x8_tab", "nut_M2.5_tab", "nut_M3_rod", "screw_M3x8_pivot", "side_plate")) else k
    if re.fullmatch(r"finger_\d", name): k = "finger"
    if re.fullmatch(r"pivot_spacer_\d", name): k = "pivot_spacer"
    return k

def color_of(name):
    for k, c in COLOR:
        if k in name: return c
    return (0.12, 0.12, 0.12)

def load(p):
    sub = "parts_print_step_stl" if p["kind"] == "print" else "parts_hardware_reference"
    return cq.importers.importStep(os.path.join(ROOT, "cad", sub, p["name"] + ".step")).val()

import itertools, numpy as np
ROTS = []   # the 24 axis-aligned rotation matrices (det = +1)
for perm in itertools.permutations(range(3)):
    for signs in itertools.product((1, -1), repeat=3):
        M = np.zeros((3, 3))
        for r, (c, sg) in enumerate(zip(perm, signs)): M[r, c] = sg
        if round(np.linalg.det(M)) == 1 and np.allclose(M, M.T): ROTS.append(M)   # identity / 180-deg flips only: symmetric, so row- vs column-vector conventions agree
ROTS.sort(key=lambda M: -np.trace(M))        # identity first

def vkey(pts):
    return sorted(tuple(round(float(v), 3) + 0.0 for v in q) for q in pts)

def match(proto, other):
    """Return (M, t) with other == M @ proto + t (exact vertex sets, equal volume and area), else None."""
    if abs(proto.Volume() - other.Volume()) > 1e-3 or abs(proto.Area() - other.Area()) > 1e-3: return None
    P = np.array([v.toTuple() for v in proto.Vertices()]); O = np.array([v.toTuple() for v in other.Vertices()])
    if len(P) != len(O): return None
    ok = vkey(O)
    for M in ROTS:
        Q = P @ M.T
        t = O.min(axis=0) - Q.min(axis=0)
        if vkey(Q + t) == ok: return M, t
    return None

# ---- group instances under unique parts (identity checked geometrically, not just by name)
uniques = []   # dict(name, shape, kind, instances=[(instance_name, Vector)])
for p in manifest:
    shp = load(p); nice = NICE[key_of(p["name"])]
    placed = False
    for u in uniques:
        if u["nice"] != nice: continue
        mt = match(u["shape"], shp)
        if mt is not None:
            u["instances"].append((p["name"], mt)); placed = True; break
    if not placed:
        n = sum(1 for u in uniques if u["nice"] == nice)
        uniques.append(dict(nice=nice, name=nice if n == 0 else f"{nice} {chr(65 + n)}", shape=shp, kind=p["kind"],
                            color=color_of(p["name"]), instances=[(p["name"], (np.eye(3), np.zeros(3)))]))

# ---- write one STEP per unique part
def write_part(path, shape, name, color):
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()); ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    lab = st.AddShape(shape.wrapped, False); TDataStd_Name.Set_s(lab, TCollection_ExtendedString(name))
    ct.SetColor(lab, Quantity_Color(*color, Quantity_TOC_RGB), XCAFDoc_ColorType.XCAFDoc_ColorSurf)
    w = STEPCAFControl_Writer(); w.SetNameMode(True); w.SetColorMode(True)
    Interface_Static.SetCVal_s("write.step.unit", "MM"); Interface_Static.SetCVal_s("write.step.schema", "AP214IS")
    w.Transfer(doc, STEPControl_AsIs); assert w.Write(path) == IFSelect_RetDone

for u in uniques:
    write_part(os.path.join(SW, "parts", u["name"] + ".step"), u["shape"], u["name"], u["color"])

# ---- instanced assembly STEP
doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()); ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
asm = st.NewShape(); TDataStd_Name.Set_s(asm, TCollection_ExtendedString("Myoelectric Hand Assembly"))
for u in uniques:
    pl = st.AddShape(u["shape"].wrapped, False); TDataStd_Name.Set_s(pl, TCollection_ExtendedString(u["name"]))
    ct.SetColor(pl, Quantity_Color(*u["color"], Quantity_TOC_RGB), XCAFDoc_ColorType.XCAFDoc_ColorSurf)
    for i, (inst, d) in enumerate(u["instances"]):
        M, tv = d
        t = gp_Trsf(); t.SetValues(*M[0], tv[0], *M[1], tv[1], *M[2], tv[2])
        cl = st.AddComponent(asm, pl, TopLoc_Location(t))
        TDataStd_Name.Set_s(cl, TCollection_ExtendedString(f"{u['name']}-{i+1}"))
st.UpdateAssemblies()
w = STEPCAFControl_Writer(); w.SetNameMode(True); w.SetColorMode(True)
Interface_Static.SetCVal_s("write.step.unit", "MM"); Interface_Static.SetCVal_s("write.step.schema", "AP214IS")
w.Transfer(doc, STEPControl_AsIs); assert w.Write(os.path.join(SW, "Myoelectric Hand Assembly.step")) == IFSelect_RetDone

# ---- instance table (used by the macro) and summary
with open(os.path.join(SW, "instances.csv"), "w", newline="") as f:
    cw = csv.writer(f)
    cw.writerow(["part_file", "instance", "r11", "r12", "r13", "r21", "r22", "r23", "r31", "r32", "r33", "x_mm", "y_mm", "z_mm", "type"])
    for u in uniques:
        for i, (inst, (M, tv)) in enumerate(u["instances"]):
            cw.writerow([u["name"], f"{u['name']}-{i+1}"] + [f"{v:.0f}" for v in M.flatten()] + [f"{v:.6f}" for v in tv]
                        + ["printed" if u["kind"] == "print" else "hardware"])
json.dump([dict(part=u["name"], qty=len(u["instances"]), kind=u["kind"], from_=[i for i, _ in u["instances"]]) for u in uniques],
          open(os.path.join(SW, "parts_summary.json"), "w"), indent=1)
print(f"unique parts: {len(uniques)}  instances: {sum(len(u['instances']) for u in uniques)}")
for u in uniques: print(f"  {len(u['instances'])} x {u['name']}  <- {[i for i, _ in u['instances']]}")
