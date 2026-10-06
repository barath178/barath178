import sys, os, json, math, cadquery as cq
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
D=sys.argv[1]; m=json.load(open(os.path.join(D,'manifest.json')))
shp={p['name']:cq.importers.importStep(os.path.join(D,'parts_print' if p['kind']=='print' else 'parts_hardware_reference',p['name']+'.step')).val() for p in m}
P=(-17.64,-17.15); B0=(-9.87,-27.7); r1=5.0; t0=math.radians(72); C0=(r1*math.cos(t0),r1*math.sin(t0))
L=math.dist(C0,B0); r3=math.dist(P,B0); b0=math.atan2(B0[1]-P[1],B0[0]-P[0])
def ph(th):
    C=(r1*math.cos(th),r1*math.sin(th)); d=math.dist(C,P)
    base=math.atan2(C[1]-P[1],C[0]-P[0]); a=math.acos(max(-1,min(1,(r3**2+d**2-L**2)/(2*r3*d)))); return base,a,C
bb,a,_=ph(t0); s=-1 if abs(((bb-a)-b0+math.pi)%(2*math.pi)-math.pi)<1e-6 else 1
fing=[n for n in shp if n.startswith('finger_') and 'tube' not in n]+[n for n in shp if n.startswith('rod_spacer') or 'coupling_rod' in n or n.startswith('nut_M3_rod')]
crank=['crank_arm','crank_pin_D3x4']; link=['drive_link']
static=[n for n in shp if n not in fing+crank+link]
def vol(a,b):
    ba,bb=a.BoundingBox(),b.BoundingBox()
    if ba.xmax<bb.xmin or bb.xmax<ba.xmin or ba.ymax<bb.ymin or bb.ymax<ba.ymin or ba.zmax<bb.zmin or bb.zmax<ba.zmin: return 0
    return cq.Shape.cast(BRepAlgoAPI_Common(a.wrapped,b.wrapped).Shape()).Volume()
Y=cq.Vector(0,1,0); total=0
for k in range(0,360,10):
    th=t0+math.radians(k); bb,a,C=ph(th); phi=bb+s*a
    B=(P[0]+r3*math.cos(phi),P[1]+r3*math.sin(phi))
    dphi=math.degrees(phi-b0)                     # CCW in the x-z view
    pv=cq.Vector(P[0],0,P[1])
    moved={n:shp[n].rotate(pv,pv+Y,-dphi) for n in fing}
    o=cq.Vector(0,0,0)
    for n in crank: moved[n]=shp[n].rotate(o,o+Y,-k)
    # link: rigid motion taking (C0,B0) -> (C,B)
    ang=math.degrees(math.atan2(B[1]-C[1],B[0]-C[0])-math.atan2(B0[1]-C0[1],B0[0]-C0[0]))
    l=shp['drive_link'].rotate(cq.Vector(C0[0],0,C0[1]),cq.Vector(C0[0],1,C0[1]),-ang).translate(cq.Vector(C[0]-C0[0],0,C[1]-C0[1]))
    moved['drive_link']=l
    hits=[]
    for mn,ms in moved.items():
        for sn in static:
            v=vol(ms,shp[sn])
            if v>0.05 and not ('screw' in sn and 'tube' in mn) and not (mn=='crank_arm' and sn.startswith('n20')): hits.append(f"{mn} x {sn}: {v:.2f}")
    for a_,b_ in (('drive_link','finger_1'),('drive_link','crank_arm'),('crank_arm','nut_M3_rod_front'),('drive_link','nut_M3_rod_front')):
        v=vol(moved[a_],moved[b_]); 
        if v>0.05: hits.append(f"{a_} x {b_}: {v:.2f}")
    total+=len(hits)
    print(f"crank {k:3d} deg -> fingers open {-dphi:6.1f} deg : {'clear' if not hits else hits}")
print("total collisions:", total)
