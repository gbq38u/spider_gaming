"""Mesh integrity and nominal component clearance checks for the controller."""
from collections import Counter
from pathlib import Path
import struct
import numpy as np
import cadquery as cq
from build_remote import enclosure,lid,joystick_coupon,box,STL,STICK_X,STICK_Y

body=enclosure();cover=lid();joystick_coupon()
assert body.intersect(cover).val().Volume()<.01,"lid/body collision"

# Nominal component envelopes are deliberately not bundled as printable parts.
holder=box(21.1,77.1,21.5,0,0,2.4)
assert body.intersect(holder).val().Volume()<.01,"18650 holder strikes shell"
for x in STICK_X:
    pcb=box(26.3,34,1.6,x,STICK_Y,25.4)
    assert body.intersect(pcb).val().Volume()<.01,"joystick PCB strikes shell"
    # The four board screws run from the rear into the four bosses.
    for dx in (-10.15,10.15):
        for dy in (-13.1,13.1):
            q=cq.Vector(x+dx,STICK_Y+dy,32.0)
            assert not body.val().isInside(q,1e-6),"joystick screw pilot blocked"

for p in sorted(STL.glob("*.stl")):
    raw=p.read_bytes(); count=struct.unpack_from("<I",raw,80)[0]
    assert len(raw)==84+50*count,(p.name,"bad STL length")
    vals=np.frombuffer(raw,dtype=np.dtype([("n","<f4",(3,)),("v","<f4",(3,3)),("attr","<u2")]),count=count,offset=84)
    verts=np.round(vals["v"].reshape(-1,3),5)
    ids={}; indexed=[]
    for v in verts:
        key=tuple(v)
        if key not in ids: ids[key]=len(ids)
        indexed.append(ids[key])
    edges=Counter()
    for a,b,c in np.asarray(indexed).reshape(-1,3):
        for u,v in ((a,b),(b,c),(c,a)):
            edges[tuple(sorted((u,v)))]+=1
    bad=sum(v!=2 for v in edges.values())
    assert bad==0,(p.name,"non-manifold edges",bad)
    assert abs(float(verts[:,2].min()))<1e-4,(p.name,"not on print bed")
    print(p.name,"triangles",count,"nonmanifold",bad)
print("PASS: body, cover, PCB and holder have no nominal collisions; all STL meshes closed.")
