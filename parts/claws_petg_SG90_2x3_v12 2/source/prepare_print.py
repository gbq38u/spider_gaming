"""Rotate printable STLs so each lies on z=0 with its main pocket facing upward."""
from pathlib import Path
import math,struct
import numpy as np
from assemble import read_stl
ROOT=Path(__file__).parent;SRC=ROOT/'stl';DST=ROOT/'print_stl';DST.mkdir(exist_ok=True)
def rotation(name):
 if name.startswith('pitch_cradle_'):angle,axis=(-90 if name.endswith('left') else 90),'y'
 elif name.startswith('gripper_holder_'):angle,axis=138,'x'
 elif name.startswith('gear_bridge_'):angle,axis=-42,'x'
 else:return np.eye(3)
 c=math.cos(math.radians(angle));s=math.sin(math.radians(angle))
 if axis=='x':return np.array([[1,0,0],[0,c,-s],[0,s,c]])
 return np.array([[c,0,s],[0,1,0],[-s,0,c]])
for source in sorted(SRC.glob('*.stl')):
 arr=read_stl(source);R=rotation(source.stem)
 arr=arr@R.T
 minp=arr.reshape(-1,3).min(0);maxp=arr.reshape(-1,3).max(0)
 arr[:,:,0]-=(minp[0]+maxp[0])/2
 arr[:,:,1]-=(minp[1]+maxp[1])/2
 arr[:,:,2]-=minp[2]
 dest=DST/source.name
 with open(dest,'wb') as f:
  f.write(('PETG print: '+source.stem).encode().ljust(80,b' '))
  f.write(struct.pack('<I',len(arr)))
  for tri in arr:
   n=np.cross(tri[1]-tri[0],tri[2]-tri[0]);n/=np.linalg.norm(n)+1e-12
   f.write(struct.pack('<12fH',*n,*tri[0],*tri[1],*tri[2],0))
 print(dest.name,tuple(np.round(arr.reshape(-1,3).max(0)-arr.reshape(-1,3).min(0),1)))
