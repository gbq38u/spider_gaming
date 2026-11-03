"""Sample clearance of the fixed outriggers against moving upper front legs."""
from pathlib import Path
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
from assemble import read_stl,ROBOT,MOUNT_X,BASE_Y,BASE_Z

ROOT=Path(__file__).parent
robot=read_stl('/Users/gbq38u/Desktop/spider_gaming/детальки/Assembled_spider_light.STL')
v,idx=np.unique(robot.reshape(-1,3),axis=0,return_inverse=True)
idx=idx.reshape(-1,3)
r=np.concatenate([idx[:,0],idx[:,1],idx[:,2]])
c=np.concatenate([idx[:,1],idx[:,2],idx[:,0]])
_,labels=connected_components(coo_matrix((np.ones(len(r)),(r,c)),shape=(len(v),len(v))).tocsr(),directed=False)
faces=labels[idx[:,0]]

def sample(tri,pitch=2.5):
    # Triangular surface sampling, with spacing near 2.5 mm in expectation.
    a=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
    area=np.linalg.norm(a,axis=1)*.5
    count=np.maximum(1,np.ceil(area/(pitch*pitch/2)).astype(int))
    ix=np.repeat(np.arange(len(tri)),count)
    rng=np.random.default_rng(8104)
    u=np.sqrt(rng.random(len(ix)));v=rng.random(len(ix))
    return (1-u)[:,None]*tri[ix,0]+(u*(1-v))[:,None]*tri[ix,1]+(u*v)[:,None]*tri[ix,2]

for side,name,component,pivot in [(-1,'left',9,(139.0,346.0)),(1,'right',12,(217.5,346.0))]:
    base=read_stl(ROOT/'stl'/f'yaw_base_{name}.stl')+np.array([MOUNT_X[side],BASE_Y,BASE_Z])
    static=sample(base)
    tree=cKDTree(static)
    upper=robot[faces==component]@ROBOT[:3,:3].T
    moving=sample(upper)
    stats=[]
    for alpha in range(-35,56,5):
        angle=np.radians(-side*alpha)
        center=np.array(pivot)
        xy=moving[:,:2]-center
        rotated=np.empty_like(moving)
        rotated[:,0]=np.cos(angle)*xy[:,0]-np.sin(angle)*xy[:,1]+center[0]
        rotated[:,1]=np.sin(angle)*xy[:,0]+np.cos(angle)*xy[:,1]+center[1]
        rotated[:,2]=moving[:,2]
        dist,_=tree.query(rotated,workers=-1)
        stats.append((alpha,dist.min(),np.percentile(dist,0.1)))
    print(name,'outward-angle deg / min sample distance mm / p0.1 mm')
    for row in stats:print('%3d %5.2f %5.2f'%row)
