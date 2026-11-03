"""Build visual assembly STLs from the printable parts and the supplied servo model."""
from pathlib import Path
import struct,math
import numpy as np
ROOT=Path(__file__).parent;STL=ROOT/'stl';SRC=Path('/Users/gbq38u/Desktop/spider_gaming/детальки/SPIDER_LIGHT')
SPREAD=ROOT/'spider_front_legs_spread_reference.stl'
HORN=SRC/'hanger.STL'
BEARING=ROOT/'preview'/'reference_bearing_3x8x4.stl'
DX,DY,DZ=85.457,83.888,93.155
BASE_Z=20.5+DZ; BASE_Y=333.0; LIFT=32.0
MOUNT_X={-1:115.0,1:241.5}

def T(x=0,y=0,z=0):
 m=np.eye(4);m[:3,3]=x,y,z;return m
def Rx(a):
 a=math.radians(a);c=math.cos(a);s=math.sin(a);m=np.eye(4);m[:3,:3]=[[1,0,0],[0,c,-s],[0,s,c]];return m
def Ry(a):
 a=math.radians(a);c=math.cos(a);s=math.sin(a);m=np.eye(4);m[:3,:3]=[[c,0,s],[0,1,0],[-s,0,c]];return m
def Rz(a):
 a=math.radians(a);c=math.cos(a);s=math.sin(a);m=np.eye(4);m[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]];return m
# Native servo (x,y,z) -> pocket coordinates (x-16.25,z-6,y+2).
NATIVE=np.array([[1,0,0,-16.25],[0,0,1,-6],[0,1,0,2],[0,0,0,1]],float)
# Reference spider native x/lateral,y/height,z/forward -> assembly x,y,z.
ROBOT=np.array([[1,0,0,0],[0,0,1,0],[0,1,0,0],[0,0,0,1]],float)
# Exact supplied hanger mesh -> arm length u, transverse v, axial thickness w.
HORN_LOCAL=np.array([[-1,0,0,18.035],[0,0,1,-3.665],[0,1,0,0],[0,0,0,1]],float)
HORN_YAW=np.array([[0,1,0,0],[-1,0,0,0],[0,0,1,0],[0,0,0,1]],float)
HORN_GRIP=np.array([[0,1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]],float)

def read_stl(path):
 b=Path(path).read_bytes();n=struct.unpack_from('<I',b,80)[0]
 if len(b)!=84+50*n:raise ValueError(f'not binary STL: {path}')
 t=np.frombuffer(b,dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]),offset=84,count=n)
 return t['v'].astype(float)
def write_stl(path,assemblies):
 tris=[]
 for source,matrix,label in assemblies:
  arr=read_stl(source)
  transformed=arr@matrix[:3,:3].T+matrix[:3,3]
  if np.linalg.det(matrix[:3,:3])<0:transformed=transformed[:,[0,2,1],:]
  tris.append((label,transformed))
 total=sum(len(t) for _,t in tris)
 with open(path,'wb') as f:
  f.write(('spider claws visual assembly '+path.stem).encode()[:80].ljust(80,b' '));f.write(struct.pack('<I',total))
  for label,arr in tris:
   for t in arr:
    n=np.cross(t[1]-t[0],t[2]-t[0]);n=n/(np.linalg.norm(n)+1e-12)
    f.write(struct.pack('<12fH',*n,*t[0],*t[1],*t[2],0))
 print(path.name,'triangles',total, 'bounds',np.round(np.min(np.concatenate([a for _,a in tris]).reshape(-1,3),axis=0),1),np.round(np.max(np.concatenate([a for _,a in tris]).reshape(-1,3),axis=0),1),flush=True)
 return tris

def spread_robot():
 """View-only pose: yaw the two complete front legs away from the claws."""
 arr=read_stl(SRC/'Assembled_spider_light.STL')@ROBOT[:3,:3].T
 vertices,idx=np.unique(arr.reshape(-1,3),axis=0,return_inverse=True)
 face_vertices=idx.reshape(-1,3)
 parent=np.arange(len(vertices))
 def find(i):
  while parent[i]!=i:
   parent[i]=parent[parent[i]];i=parent[i]
  return i
 for a,b,c in face_vertices:
  ra=find(a);rb=find(b);rc=find(c)
  parent[rb]=ra;parent[rc]=ra
 labels=np.array([find(a) for a in face_vertices[:,0]])
 for side,pivot,angle in [(-1,(139.0,346.0),50.0),(1,(217.5,346.0),-30.0)]:
  chosen=[]
  for label in np.unique(labels):
   mask=labels==label
   points=arr[mask].reshape(-1,3)
   lo=points.min(0);hi=points.max(0)
   if ((side==-1 and hi[0]<160) or (side==1 and lo[0]>200)) and lo[1]>340 and hi[1]>390:
    chosen.append(label)
  mask=np.isin(labels,chosen)
  xy=arr[mask,:,:2]-np.array(pivot)
  a=math.radians(angle);c=math.cos(a);s=math.sin(a)
  arr[mask,:,0]=c*xy[:,:,0]-s*xy[:,:,1]+pivot[0]
  arr[mask,:,1]=s*xy[:,:,0]+c*xy[:,:,1]+pivot[1]
  print('spread front leg',side,'shells',len(chosen),'faces',int(mask.sum()),flush=True)
 with open(SPREAD,'wb') as f:
  f.write(b'VIEW ONLY spread front legs'.ljust(80,b' '));f.write(struct.pack('<I',len(arr)))
  for tri in arr:
   n=np.cross(tri[1]-tri[0],tri[2]-tri[0]);n=n/(np.linalg.norm(n)+1e-12)
   f.write(struct.pack('<12fH',*n,*tri[0],*tri[1],*tri[2],0))
 return SPREAD

def assemble(pitch):
 output=[];color=[]
 def add(path,m,label):output.append((path,m,label))
 for side,name in [(-1,'left'),(1,'right')]:
  arm=-side
  gb=T(MOUNT_X[side],BASE_Y,BASE_Z)
  # The raised front joint clears the leg in the straight-ahead pose.
  yaw=0
  gm=gb@T(0,12,0)@Rz(-side*yaw)@T(0,-12,0)
  # Boom coordinates already include the 20 mm shaft offset in X.
  bp=T(0,28,44+LIFT)@Rx(pitch)
  grip_x=(-31.35 if arm==-1 else 27.35)+2+arm*9
  holder=bp@T(grip_x,160,0)@Rx(42)@Ry(180)
  add(STL/f'yaw_base_{name}.stl',gb,'yaw_base_'+name)
  add(STL/f'pitch_cradle_{name}.stl',gm,'pitch_cradle_'+name)
  for key in ('boom','gripper_holder','gear_bridge'):
   add(STL/f'{key}_{name}.stl',gm@bp,key+'_'+name)
  # Gear faces sit 1 mm below the support bridge; centre distance is 20 mm.
  for key,shift in [('jaw_carrier',0),('jaw_driven',0),('jaw_passive',arm*20)]:
   add(STL/f'{key}_{name}.stl',gm@holder@T(shift,0,4.75),key+'_'+name)
  # All three servos use the exact user-provided STL, mapped into matching sockets.
  yaw_pose=(Rz(180) if side==-1 else np.eye(4))
  add(SRC/'servo.STL',gb@T(side*5.45,12,7+LIFT)@yaw_pose@NATIVE,'servo_yaw_'+name)
  pitch_pose=(Rz(180) if arm==1 else np.eye(4))
  add(SRC/'servo.STL',gm@T(-arm*10.35,28,49.45+LIFT)@Ry(arm*90)@pitch_pose@NATIVE,'servo_pitch_'+name)
  grip_pose=(Rz(180) if arm==1 else np.eye(4))
  horn_x=arm*5.45
  add(SRC/'servo.STL',gm@bp@T(grip_x,160,0)@Rx(42)@Ry(180)@T(-horn_x,0,-30.35)@grip_pose@NATIVE,'servo_grip_'+name)
  # View-only hardware: the exact stock one-arm horn and 3x8x4 bearings.
  pitch_horn=np.array([[0,0,arm,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]],float)
  add(HORN,gm@T(0,12,39.8+LIFT)@HORN_YAW@HORN_LOCAL,'horn_yaw_'+name)
  add(HORN,gm@T(arm*23,28,44+LIFT)@pitch_horn@HORN_LOCAL,'horn_pitch_'+name)
  add(HORN,gm@holder@T(0,0,.75)@HORN_GRIP@HORN_LOCAL,'horn_grip_'+name)
  add(BEARING,gm@T(0,12,-5.3+LIFT),'bearing_yaw_'+name)
  pitch_bearing_x=-20 if arm==1 else 16
  add(BEARING,gm@bp@T(pitch_bearing_x,0,0)@Ry(90),'bearing_pitch_'+name)
  add(BEARING,gm@holder@T(0,0,15.95),'bearing_grip_'+name)
  add(BEARING,gm@holder@T(arm*20,0,6.65),'bearing_passive_jaw_'+name)
 return output

if __name__=='__main__':
 spread_robot()
 for angle,title in [(-73,'work'),(20,'walk')]:
  ass=assemble(angle)
  write_stl(ROOT/f'claws_assembly_{title}.stl',ass)
  if title=='work':
   for name in ('left','right'):
    write_stl(ROOT/f'claw_{name}_assembly_work.stl',[a for a in ass if a[2].endswith('_'+name)])
   write_stl(ROOT/'spider_with_claws_reference.stl',[(SPREAD,np.eye(4),'spider')]+ass)
   write_stl(ROOT/'spider_with_claws_unspread_reference.stl',[(SRC/'Assembled_spider_light.STL',ROBOT,'spider')]+ass)
