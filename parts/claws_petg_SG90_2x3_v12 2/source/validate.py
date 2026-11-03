from pathlib import Path
from collections import Counter
import numpy as np
from assemble import read_stl
ROOT=Path(__file__).parent
for path in sorted((ROOT/'stl').glob('*.stl'))+sorted((ROOT/'print_stl').glob('*.stl')):
 a=read_stl(path);pts=np.round(a.reshape(-1,3),5)
 ids={};seq=[]
 for p in pts:
  t=tuple(p)
  if t not in ids:ids[t]=len(ids)
  seq.append(ids[t])
 tri=np.asarray(seq).reshape(-1,3)
 edge=Counter()
 for a,b,c in tri:
  for u,v in ((a,b),(b,c),(c,a)):
   edge[tuple(sorted((u,v)))]+=1
 bad=sum(1 for v in edge.values() if v!=2)
 xyz=pts.reshape(-1,3,3)
 vol=np.einsum('ij,ij->i',xyz[:,0,:],np.cross(xyz[:,1,:],xyz[:,2,:])).sum()/6
 print(path.parent.name+'/'+path.name,'tri',len(tri),'open/nonmanifold',bad,'signed mm3',round(vol,1))
 if bad or vol<=0:raise SystemExit('INVALID '+path.name)
