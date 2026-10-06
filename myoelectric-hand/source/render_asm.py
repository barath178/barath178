import sys, os, json, numpy as np, cadquery as cq
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
D=sys.argv[1]; m=json.load(open(os.path.join(D,'manifest.json')))
src=open(sys.argv[3]).read()
col={}
import re
for name,c in re.findall(r'add\(f?"([^"]+)".*?\((0?\.\d+|1\.0|0), ?(?:0?\.\d+|1\.0|0), ?(?:0?\.\d+|1\.0|0)\)', src): pass
cmap={'side_plate':(0.85,0.1,0.1),'finger':(0.93,0.83,0.55),'thumb_rocker':(1,0.55,0.05),'thumb':(0.93,0.83,0.55),'base':(0.85,0.72,0.5),'puck':(0.85,0.1,0.1),
      'n20':(0.3,0.75,0.4),'socket':(0.88,0.88,0.9),'enclosure':(0.35,0.35,0.38),'lid':(0.35,0.35,0.38),'screw':(0.2,0.8,0.3),'pin':(0.2,0.8,0.3)}
def color(n):
    for k,v in cmap.items():
        if k in n: return v
    return (0.15,0.15,0.15)
def tris_of(names):
    T=[];C=[]
    for p in m:
        if p['name'] not in names: continue
        sub='parts_print' if p['kind']=='print' else 'parts_hardware_reference'
        s=cq.importers.importStep(os.path.join(D,sub,p['name']+'.step')).val()
        v,t=s.tessellate(0.15,0.3); v=np.array([(q.x,q.y,q.z) for q in v])
        for tr in t: T.append(v[list(tr)]); C.append(color(p['name']))
    return T,C
def draw(names,views,fn,title):
    T,C=tris_of(names); allp=np.concatenate(T)
    fig=plt.figure(figsize=(6*len(views),7))
    light=np.array([0.4,-0.6,0.7]); light/=np.linalg.norm(light)
    shade=[]
    for tr,c in zip(T,C):
        n=np.cross(tr[1]-tr[0],tr[2]-tr[0]); nn=np.linalg.norm(n); k=0.55+0.45*abs(n@light/nn) if nn>0 else 1
        shade.append(tuple(min(1,x*k) for x in c))
    for i,(el,az) in enumerate(views):
        ax=fig.add_subplot(1,len(views),i+1,projection='3d')
        ax.add_collection3d(Poly3DCollection(T,facecolors=shade,edgecolor='none'))
        mn,mx=allp.min(0),allp.max(0); c=(mn+mx)/2; r=(mx-mn).max()/2
        ax.set_xlim(c[0]-r,c[0]+r);ax.set_ylim(c[1]-r,c[1]+r);ax.set_zlim(c[2]-r,c[2]+r)
        ax.view_init(el,az); ax.set_box_aspect((1,1,1)); ax.axis('off')
    fig.suptitle(title); plt.tight_layout(); plt.savefig(fn,dpi=90); plt.close()
hand=[p['name'] for p in m if not any(k in p['name'] for k in ('socket','enclosure','lid','battery','arduino','driver','emg'))]
# Z up is towards the forearm; flip with negative elevation views so the hand looks like the screenshots
draw(hand,[(-25,-60),(-25,-120),(0,-90)],sys.argv[2]+'_hand.png','Hand assembly (fingers down = image flipped vs screenshots)')
draw([p['name'] for p in m],[(-20,-50),(15,-130)],sys.argv[2]+'_full.png','Complete assembly with forearm socket + electronics')
