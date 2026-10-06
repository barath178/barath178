import sys, os, json, itertools, math, cadquery as cq, trimesh
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
D=sys.argv[1]; m=json.load(open(os.path.join(D,'manifest.json')))
shp={}; bad=0
for p in m:
    sub='parts_print' if p['kind']=='print' else 'parts_hardware_reference'
    s=cq.importers.importStep(os.path.join(D,sub,p['name']+'.step')).val(); shp[p['name']]=s
    ok=s.isValid() and len(s.Solids())==1
    extra=''
    if p['kind']=='print':
        t=trimesh.load(os.path.join(D,sub,p['name']+'.stl'))
        ok = ok and t.is_watertight
        extra=f" stl_watertight={t.is_watertight} stl_vol={t.volume:.1f}"
    if not ok: bad+=1
    print(f"{'OK ' if ok else 'BAD'} {p['kind']:5} {p['name']:38} vol={s.Volume():9.1f} size={p['size']}{extra}")
print("invalid parts:", bad)
def vol(a,b):
    ba,bb=a.BoundingBox(),b.BoundingBox()
    if ba.xmax<bb.xmin or bb.xmax<ba.xmin or ba.ymax<bb.ymin or bb.ymax<ba.ymin or ba.zmax<bb.zmin or bb.zmax<ba.zmin: return 0
    return cq.Shape.cast(BRepAlgoAPI_Common(a.wrapped,b.wrapped).Shape()).Volume()
THREAD=lambda a,b: {a,b}=={'crank_arm','n20_gearmotor_6V_ref'} or (('screw' in a or 'rod' in a) and any(k in b for k in ('puck','tube','enclosure','n20','nut'))) or (('screw' in b or 'rod' in b) and any(k in a for k in ('puck','tube','enclosure','n20','nut')))
print("\nCONTACT / INTERFERENCE (>0.05 mm3):")
real=0
for a,b in itertools.combinations(shp,2):
    v=vol(shp[a],shp[b])
    if v>0.05:
        tag='thread engagement (expected)' if THREAD(a,b) else 'INTERFERENCE'
        if tag=='INTERFERENCE': real+=1
        print(f"  {tag:30} {a} x {b}: {v:.2f}")
print("real interferences:", real)
# finger sweep: rotate fingers + rod + rod spacers about the pivot, check vs static parts
piv=cq.Vector(-17.64,0,-17.15)
moving=[n for n in shp if n.startswith('finger_') and 'tube' not in n]+[n for n in shp if n.startswith('rod_spacer') or 'coupling_rod' in n or n.startswith('nut_M3_rod')]
static=[n for n in shp if n not in moving and n not in ('drive_link','crank_arm','crank_pin_D3x4') and 'pivot' not in n]
print("\nFINGER SWEEP (fingers rotate about the pivot tube):")
for ang in (-40,-30,-20,-10,10,20,30):
    hits=[]
    for mn in moving:
        r=shp[mn].rotate(piv,piv+cq.Vector(0,1,0),ang)
        for sn in static:
            v=vol(r,shp[sn])
            if v>0.05: hits.append(f"{mn}x{sn}:{v:.1f}")
    print(f"  {ang:+d} deg: {'clear' if not hits else hits}")
