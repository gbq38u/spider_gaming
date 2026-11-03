"""Software previews of the printable controller enclosure."""
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
import cadquery as cq
from build_remote import enclosure,lid,box,cylinder

ROOT=Path(__file__).parent
PREVIEW=ROOT/"preview";PREVIEW.mkdir(exist_ok=True)
body=enclosure();cover=lid()

def triangles(shape):
    vertices,faces=shape.val().tessellate(.12)
    v=np.array([p.toTuple() for p in vertices],dtype=float)
    return v[np.asarray(faces,dtype=int)]

def draw(path,parts,eye,center,span=200,title=""):
    width,height=1400,1000
    e=np.array(eye,float);e/=np.linalg.norm(e)
    up=np.array([0.,0.,1.])
    right=np.cross(up,e)
    if np.linalg.norm(right)<.01:right=np.array([1.,0.,0.])
    right/=np.linalg.norm(right)
    vert=np.cross(e,right);vert/=np.linalg.norm(vert)
    center=np.array(center,float)
    faces=[]
    for shape,offset,color in parts:
        a=triangles(shape)+np.array(offset)
        q=np.stack([(a-center)@right,(a-center)@vert,(a-center)@e],axis=-1)
        n=np.cross(q[:,1]-q[:,0],q[:,2]-q[:,0])
        shade=.74+.26*np.abs(n[:,2])/(np.linalg.norm(n,axis=1)+1e-10)
        for tri,s in zip(q,shade):
            xy=tri[:,:2]*((height-100)/span)+[width/2,height/2]
            xy[:,1]=height-xy[:,1]
            rgb=tuple(int(min(255,v*s)) for v in color)
            faces.append((tri[:,2].mean(),[tuple(p) for p in xy],rgb))
    faces.sort(key=lambda f:f[0])
    im=Image.new("RGB",(width,height),(244,246,248))
    d=ImageDraw.Draw(im)
    for _,xy,col in faces:d.polygon(xy,fill=col)
    d.text((36,30),title,fill=(22,33,39))
    im.save(path)
    print(path.name,len(faces))

draw(PREVIEW/"controller_top.png",[(body,(0,0,0),(34,121,154)),
     (cover,(0,0,0),(35,90,114))],(.36,-.55,1),(0,0,20),185,
     "TWIN STICK CONTROLLER | top")
draw(PREVIEW/"controller_open_back.png",[(body,(0,0,0),(34,121,154)),
     (cover,(0,-115,-15),(245,142,59))],(.25,-.35,-1),(0,-35,8),205,
     "OPEN BACK | battery tray on snap cover")
draw(PREVIEW/"controller_lid.png",[(cover,(0,0,0),(245,142,59))],
     (.65,-.6,1),(0,0,4),180,"SNAP COVER | single 18650 holder bay")

# Reference hardware silhouettes for scale only; these are not printable STL.
hardware=[]
for x in (-39,39):
    collar=cylinder(26.2,6,x,8,40)
    stem=cylinder(7,9,x,8,45)
    cap=cq.Workplane("XY").sphere(10).translate((x,8,55))
    hardware.extend([(collar,(0,0,0),(50,54,60)),
                     (stem,(0,0,0),(38,40,44)),
                     (cap,(0,0,0),(28,29,33))])
hardware.extend([(box(15,10,3,62,-15,40),(0,0,0),(34,36,41)),
                 (cylinder(5,3,0,-28,40),(0,0,0),(72,211,125))])
draw(PREVIEW/"controller_with_hardware_concept.png",
     [(body,(0,0,0),(34,121,154))]+hardware,
     (.36,-.55,1),(0,0,24),190,"CONTROLLER | concept with typical stick caps")
