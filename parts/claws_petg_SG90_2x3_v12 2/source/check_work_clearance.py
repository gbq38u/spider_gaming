"""Approximate surface clearance in the rendered working pose (millimetres)."""
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
from assemble import read_stl, SRC, SPREAD, assemble, MOUNT_X, BASE_Y

original = read_stl(SRC / 'Assembled_spider_light.STL')
vertices, index = np.unique(original.reshape(-1, 3), axis=0, return_inverse=True)
index = index.reshape(-1, 3)
row = np.concatenate([index[:, 0], index[:, 1], index[:, 2]])
col = np.concatenate([index[:, 1], index[:, 2], index[:, 0]])
_, labels = connected_components(
    coo_matrix((np.ones(len(row)), (row, col)), shape=(len(vertices), len(vertices))).tocsr(),
    directed=False,
)
face_group = labels[index[:, 0]]
spread = read_stl(SPREAD)  # same triangle order as original
assembly = assemble(-73)

def samples(triangles):
    return np.concatenate((triangles.reshape(-1, 3), triangles.mean(axis=1)), axis=0)

for side, name, leg_group in [(-1, 'left', 9), (1, 'right', 12)]:
    tree = cKDTree(samples(spread[face_group == leg_group]))
    print(name)
    for yaw in (0,5,10,15):
        closest=(float('inf'),'')
        for path, matrix, label in assembly:
            if not label.endswith('_' + name) or label.startswith(('servo', 'horn', 'bearing')):
                continue
            triangles = read_stl(path) @ matrix[:3, :3].T + matrix[:3, 3]
            if label.startswith('yaw_base'):
                continue
            centre=np.array([MOUNT_X[side],BASE_Y+12.0])
            xy=triangles[:,:,:2]-centre
            a=np.deg2rad(-side*yaw);c=np.cos(a);s=np.sin(a)
            triangles[:,:,0]=c*xy[:,:,0]-s*xy[:,:,1]+centre[0]
            triangles[:,:,1]=s*xy[:,:,0]+c*xy[:,:,1]+centre[1]
            distances = tree.query(samples(triangles), workers=-1)[0]
            if distances.min()<closest[0]:closest=(float(distances.min()),label)
        print('outward yaw',yaw,'closest',closest[1],round(closest[0],2),'mm sampled')
