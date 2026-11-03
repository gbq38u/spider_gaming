"""Software render for human inspection; assembly meshes retain original millimetres."""
import sys,math
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from assemble import assemble,read_stl,ROBOT,SRC,SPREAD
ROOT=Path(__file__).parent;P=ROOT/'preview';P.mkdir(exist_ok=True)
COLORS={
 'spider':(183,187,192),'yaw_base':(32,92,144),'pitch_cradle':(55,125,181),
 'boom':(57,144,190),'gripper_holder':(28,117,144),'gear_bridge':(20,112,130),'jaw_driven':(223,108,49),
 'jaw_carrier':(172,90,46),
 'jaw_passive':(237,148,63),'servo':(50,52,62)
 ,'horn':(224,224,213),'bearing':(120,129,137)
}
def draw_view(path,items,eye,center,width=1650,height=1200,title='',span_mm=None):
 e=np.array(eye,float);e/=np.linalg.norm(e)
 up=np.array([0.,0.,1.]);right=np.cross(up,e)
 if np.linalg.norm(right)<1e-8:right=np.array([1.,0.,0.])
 else:right/=np.linalg.norm(right)
 vert=np.cross(e,right);vert/=np.linalg.norm(vert)
 projected=[];bounds=[]
 for src,m,label in items:
  a=read_stl(src)@m[:3,:3].T+m[:3,3]
  p=np.stack([(a-center)@right,(a-center)@vert,(a-center)@e],axis=-1)
  projected.append((p,label))
  bounds.append(p[:,:,:2].reshape(-1,2))
 b=np.concatenate(bounds);lo=b.min(0);hi=b.max(0);span=hi-lo
 if span_mm is not None:
  span=np.array([span_mm*width/height,span_mm],float)
  lo=-span/2
 scale=min((width-150)/span[0],(height-150)/span[1])
 offset=np.array([(width-span[0]*scale)/2,(height-span[1]*scale)/2])
 # Far triangles first.
 faces=[]
 for p,label in projected:
  key=next((k for k in COLORS if label.startswith(k)), 'spider')
  rgb=np.array(COLORS[key],float)
  a=p[:,1,:]-p[:,0,:];c=p[:,2,:]-p[:,0,:]
  normal=np.cross(a,c)
  shade=np.clip(0.73+0.27*np.abs(normal[:,2])/(np.linalg.norm(normal,axis=1)+1e-8),0.65,1)
  face_rgb=np.clip(rgb[None,:]*shade[:,None],0,255).astype(np.uint8)
  for tri,col in zip(p,face_rgb):
   xy=(tri[:,:2]-lo)*scale+offset
   xy[:,1]=height-xy[:,1]
   faces.append((float(tri[:,2].mean()),[tuple(v) for v in xy],tuple(map(int,col))))
 faces.sort(key=lambda a:a[0])
 im=Image.new('RGB',(width,height),(248,249,250));d=ImageDraw.Draw(im)
 for _,xy,col in faces:d.polygon(xy,fill=col)
 d.text((28,24),title,fill=(28,35,43))
 im.save(path)
 print(path.name, len(faces),flush=True)
for angle,title in [(-73,'work'),(20,'walk')]:
 items=assemble(angle)
 if title=='work':
  items=[(SPREAD,np.eye(4),'spider')]+items
  draw_view(P/'work_top.png',items,(0,0,1),np.array([178,245,85.]),title='WORK POSE  |  TOP VIEW  |  front legs spread')
  draw_view(P/'work_side.png',items,(1,0.2,0.2),np.array([178,245,85.]),title='WORK POSE  |  SIDE VIEW  |  raised joint above front leg')
 draw_view(P/(title+'_iso.png'),items,(1.0,0.9,1.2),np.array([178,245,85.]),title=title.upper()+' POSE  |  SG90 METAL-GEAR x6')
