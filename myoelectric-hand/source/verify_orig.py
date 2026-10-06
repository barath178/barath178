import sys, os, cadquery as cq
from OCP.STEPControl import STEPControl_Reader
rd=STEPControl_Reader(); rd.ReadFile(sys.argv[1]); rd.TransferRoots(); sh=cq.Shape.cast(rd.Shape(1)).Shells()
D=sys.argv[2]
def bbx(s): b=s.BoundingBox(); return (b.xmin,b.xmax,b.zmin,b.zmax)
def load(n): return cq.importers.importStep(os.path.join(D,'parts_print',n+'.step')).val()
pairs=[('finger_1 vs Component75',sh[0],load('finger_1')),('thumb vs Component77',sh[2],load('thumb')),
       ('side_plate_front vs Component78',sh[3],load('side_plate_front'))]
for n,o,new in pairs:
    a,b=bbx(o),bbx(new); print(f"{n}: orig x{a[0]:.2f}..{a[1]:.2f} z{a[2]:.2f}..{a[3]:.2f} | new x{b[0]:.2f}..{b[1]:.2f} z{b[2]:.2f}..{b[3]:.2f} | max diff {max(abs(p-q) for p,q in zip(a,b)):.3f} mm")
# every original hole circle must exist in the new part (same centre & radius)
def circles(s):
    out=set()
    for e in s.Edges():
        if e.geomType()=='CIRCLE' and abs(e.Length()-2*3.14159265*e.radius())<1e-3:
            c=e.arcCenter(); out.add((round(c.x,2),round(c.z,2),round(e.radius(),2)))
    return out
for n,o,new in pairs:
    oc=circles(o); nc=circles(new); miss=[c for c in oc if c not in nc]
    print(f"  full-circle holes in original: {sorted(oc)}  missing in new: {miss}")
tab=load('base_bracket_with_motor_housing'); print('tab holes present:', {(-7.0,19.9,1.35),(11.0,19.9,1.35)} <= circles(tab))
pk=load('wrist_puck'); b=pk.BoundingBox(); print(f"puck: x{b.xmin:.2f}..{b.xmax:.2f} y{b.ymin:.2f}..{b.ymax:.2f} z{b.zmin:.2f}..{b.zmax:.2f} (orig r25 about (0,-23), z30..47.3)", 'bore r9.1:', (0.0,-23.0) and any(abs(e.radius()-9.1)<1e-3 for e in pk.Edges() if e.geomType()=='CIRCLE'))
