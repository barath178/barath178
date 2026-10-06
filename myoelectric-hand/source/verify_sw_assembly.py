"""Re-read 'Myoelectric Hand Assembly.step' and check every instance lands exactly on the original part."""
import sys, os, json, cadquery as cq
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
R = sys.argv[1]
doc = TDocStd_Document(TCollection_ExtendedString("d"))
rd = STEPCAFControl_Reader(); rd.SetNameMode(True); rd.ReadFile(os.path.join(R, "solidworks", "Myoelectric Hand Assembly.step")); rd.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
def nm(l):
    a = TDataStd_Name(); return a.Get().ToExtString() if l.FindAttribute(TDataStd_Name.GetID_s(), a) else "?"
top = TDF_LabelSequence(); st.GetFreeShapes(top); asm = top.Value(1)
comps = TDF_LabelSequence(); st.GetComponents_s(asm, comps)
parts = set(); placed = []
for i in range(1, comps.Length() + 1):
    c = comps.Value(i); ref = TDF_Label(); st.GetReferredShape_s(c, ref); parts.add(nm(ref))
    placed.append((nm(c), cq.Shape.cast(st.GetShape_s(c))))
def key(s): return sorted(tuple(round(v, 3) + 0.0 for v in p.toTuple()) for p in s.Vertices())
orig = {}
for p in json.load(open(os.path.join(R, "cad", "manifest.json"))):
    sub = "parts_print_step_stl" if p["kind"] == "print" else "parts_hardware_reference"
    s = cq.importers.importStep(os.path.join(R, "cad", sub, p["name"] + ".step")).val()
    orig[p["name"]] = (key(s), round(s.Volume(), 2))
unmatched = dict(orig); bad = []
for n, s in placed:
    k = (key(s), round(s.Volume(), 2))
    hit = next((o for o, v in unmatched.items() if v == k), None)
    if hit: unmatched.pop(hit)
    else: bad.append(n)
print(f"assembly: '{nm(asm)}', {len(parts)} unique parts, {len(placed)} instances")
print("instances not matching an original part:", bad)
print("original parts not covered:", list(unmatched))
