import sys, os, json, math, shutil, numpy as np, trimesh
D=sys.argv[1]; m=json.load(open(os.path.join(D,'manifest.json')))
outd=os.path.join(D,'stl_print_ready'); os.makedirs(outd,exist_ok=True)
def R(axis,deg): return trimesh.transformations.rotation_matrix(math.radians(deg),axis)
X,Y=[1,0,0],[0,1,0]
orient={'base_bracket_with_motor_housing':R(X,180),'wrist_puck':R(X,180),'forearm_socket':np.eye(4),
        'electronics_enclosure':R(Y,90),'enclosure_lid':R(Y,90)}
groups={}   # identical parts -> one file with quantity
for p in m:
    if p['kind']!='print': continue
    n=p['name']
    key=('finger' if n.startswith('finger_') and 'tube' not in n else
         'side_plate' if n.startswith('side_plate') else
         'pivot_spacer_between_fingers' if n.startswith('pivot_spacer_') and n[-1].isdigit() else
         'pivot_spacer_front' if n=='pivot_spacer_front' else
         'rod_spacer' if n.startswith('rod_spacer') else n)
    if n=='pivot_spacer_back': key='pivot_spacer_back'
    groups.setdefault(key,[]).append(n)
rows=[]
for key,names in groups.items():
    n=names[0]
    t=trimesh.load(os.path.join(D,'parts_print',n+'.stl'))
    t.apply_transform(orient.get(n,R(X,-90)))        # default: plate thickness (Y) -> vertical
    t.apply_translation([-t.bounds[:,0].mean(),-t.bounds[:,1].mean(),-t.bounds[0,2]])
    fn=f"{key}_x{len(names)}.stl"; t.export(os.path.join(outd,fn))
    ext=t.extents; rows.append((fn,len(names),ext,t.volume,t.is_watertight))
for r in sorted(rows): print(f"{r[0]:48} qty {r[1]}  bed footprint {r[2][0]:.1f} x {r[2][1]:.1f}  height {r[2][2]:.1f} mm  vol {r[3]/1000:.2f} cm3  watertight={r[4]}")
json.dump([dict(file=r[0],qty=r[1],size_xyz=[round(v,2) for v in r[2]],volume_cm3=round(r[3]/1000,2)) for r in sorted(rows)],open(os.path.join(D,'print_list.json'),'w'),indent=1)
