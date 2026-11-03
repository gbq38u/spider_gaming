"""Sample exact CAD intersections throughout the intended joint motion."""
import runpy
import itertools
import cadquery as cq
from assemble import MOUNT_X, BASE_Y, BASE_Z

parts = runpy.run_path('build_claws.py')['parts']
failures=[]

def vol(a, b):
    x, y = a.val().BoundingBox(), b.val().BoundingBox()
    if x.xmax < y.xmin or y.xmax < x.xmin or x.ymax < y.ymin or y.ymax < x.ymin or x.zmax < y.zmin or y.zmax < x.zmin:
        return 0.0
    return a.intersect(b).val().Volume()

def pitch(p, angle):
    return p.rotate((0, 0, 0), (1, 0, 0), angle).translate((0, 28, 76))

def yaw(p, angle):
    return p.rotate((0, 12, 0), (0, 12, 1), angle)

for name, arm in [('left', 1), ('right', -1)]:
    base = parts['yaw_base_'+name]
    cradle = parts['pitch_cradle_'+name]
    boom = parts['boom_'+name]
    holder = parts['gripper_holder_'+name]
    bridge = parts['gear_bridge_'+name]
    driven = parts['jaw_driven_'+name]
    carrier = parts['jaw_carrier_'+name]
    passive = parts['jaw_passive_'+name]
    grip_x=(27.35 if arm == 1 else -31.35)+2+arm*9
    def jaw_local(p, shift, angle):
        return p.rotate((0,0,0),(0,0,1),angle).translate((shift,0,4.75))
    def jaw_pose(p, shift, angle):
        q=jaw_local(p,shift,angle)
        return q.rotate((0,0,0),(0,1,0),180).rotate((0,0,0),(1,0,0),42).translate((grip_x,160,0))
    print('\nSIDE', name, flush=True)
    for label,a,b in [('boom-holder',boom,holder),('holder-bridge',holder,bridge),
                      ('boom-bridge',boom,bridge)]:
        print(label,round(vol(a,b),5),flush=True)
    for angle in range(-90,91,10):
        v=vol(base,yaw(cradle,angle))
        if v>1e-4:print('YAW COLLISION',angle,round(v,3),flush=True)
        if abs(angle)<=15 and v>1e-4:failures.append((name,'yaw',angle,v))
    print('yaw sweep complete',flush=True)
    for angle in range(-80,61,5):
        qbo=pitch(boom,angle)
        qh=pitch(holder,angle)
        qbr=pitch(bridge,angle)
        for label, a,b in [('boom-cradle',cradle,qbo),('holder-cradle',cradle,qh),
                           ('bridge-cradle',cradle,qbr),('boom-base',base,qbo),
                           ('holder-base',base,qh),('bridge-base',base,qbr)]:
            v=vol(a,b)
            if v>1e-4:print('PITCH COLLISION',angle,label,round(v,3),flush=True)
            if -73<=angle<=55 and v>1e-4:failures.append((name,'pitch',angle,label,v))
    print('pitch sweep complete',flush=True)
    for angle in range(-40,41,5):
        a=jaw_pose(driven,0,angle)
        ac=jaw_pose(carrier,0,angle)
        b=jaw_pose(passive,arm*20,-angle)
        for label,x,y in [('jaws',a,b),('driven-bridge',a,bridge),
                          ('passive-bridge',b,bridge),('driven-holder',a,holder),
                          ('passive-holder',b,holder),('carrier-bridge',ac,bridge),
                          ('carrier-holder',ac,holder),('carrier-passive',ac,b),
                          ('gear-carrier',a,ac)]:
            v=vol(x,y)
            if v>1e-4:print('GRIP COLLISION',angle,label,round(v,3),flush=True)
            if 0<=angle*arm<=20 and v>1e-4:failures.append((name,'grip',angle,label,v))
    print('grip sweep complete',flush=True)

# Check simultaneous yaw and lift against each fixed socket. The two sides
# approach one another most closely with opposite yaw signs.
for yaw_left in (0,7.5,15):
  for yaw_right in (0,-7.5,-15):
    for pitch_angle in (-73,-40,0,20,55):
        posed={}
        for side,name,angle in [(-1,'left',yaw_left),(1,'right',yaw_right)]:
            shift=(MOUNT_X[side],BASE_Y,BASE_Z)
            posed[name+'_base']=parts['yaw_base_'+name].translate(shift)
            posed[name+'_cradle']=yaw(parts['pitch_cradle_'+name],angle).translate(shift)
            for key in ('boom','gripper_holder','gear_bridge'):
                posed[name+'_'+key]=yaw(pitch(parts[key+'_'+name],pitch_angle),angle).translate(shift)
        for name in ('left','right'):
            for key in ('boom','gripper_holder','gear_bridge'):
                mobile=posed[name+'_'+key]
                for fixed in ('base','cradle'):
                    v=vol(mobile,posed[name+'_'+fixed])
                    if v>1e-4:print('COMBINED COLLISION',yaw_left,yaw_right,pitch_angle,name,key,fixed,round(v,3),flush=True)
                    if v>1e-4:failures.append(('combined',yaw_left,yaw_right,pitch_angle,name,key,fixed,v))
        for a in ('boom','gripper_holder','gear_bridge'):
            for b in ('boom','gripper_holder','gear_bridge'):
                v=vol(posed['left_'+a],posed['right_'+b])
                if v>1e-4:print('OPPOSITE COLLISION',yaw_left,yaw_right,pitch_angle,a,b,round(v,3),flush=True)
                if v>1e-4:failures.append(('opposite',yaw_left,yaw_right,pitch_angle,a,b,v))
print('combined outward sweep complete',flush=True)

for yl,yr in ((0,0),(15,-15)):
    for lift in (-73,20,55):
        for opening in (0,10,20):
            posed={}
            for side,name,yaw_angle in ((-1,'left',yl),(1,'right',yr)):
                arm=-side
                grip_x=(27.35 if arm==1 else -31.35)+2+arm*9
                shift=(MOUNT_X[side],BASE_Y,BASE_Z)
                sign=arm*opening
                for key,offset,theta in (('jaw_driven',0,sign),('jaw_carrier',0,sign),('jaw_passive',arm*20,-sign)):
                    q=parts[key+'_'+name].rotate((0,0,0),(0,0,1),theta).translate((offset,0,4.75))
                    q=q.rotate((0,0,0),(0,1,0),180).rotate((0,0,0),(1,0,0),42).translate((grip_x,160,0))
                    posed[name+'_'+key]=yaw(pitch(q,lift),yaw_angle).translate(shift)
                for key in ('yaw_base','pitch_cradle','boom','gripper_holder','gear_bridge'):
                    p=parts[key+'_'+name]
                    if key=='yaw_base':p=p.translate(shift)
                    elif key=='pitch_cradle':p=yaw(p,yaw_angle).translate(shift)
                    else:p=yaw(pitch(p,lift),yaw_angle).translate(shift)
                    posed[name+'_'+key]=p
            for name,other in (('left','right'),('right','left')):
                for moving in ('jaw_driven','jaw_carrier','jaw_passive'):
                    for fixed in ('yaw_base','pitch_cradle'):
                        v=vol(posed[name+'_'+moving],posed[name+'_'+fixed])
                        if v>1e-4:print('JAW BODY COLLISION',yl,yr,lift,opening,name,moving,fixed,round(v,3),flush=True)
                        if v>1e-4:failures.append(('jaw body',yl,yr,lift,opening,name,moving,fixed,v))
                    for key in ('boom','gripper_holder','gear_bridge','jaw_driven','jaw_carrier','jaw_passive'):
                        v=vol(posed[name+'_'+moving],posed[other+'_'+key])
                        if v>1e-4:print('CROSS JAW COLLISION',yl,yr,lift,opening,name,moving,key,round(v,3),flush=True)
                        if v>1e-4:failures.append(('cross jaw',yl,yr,lift,opening,name,moving,key,v))
print('jaw workspace sweep complete',flush=True)
if failures:raise AssertionError(f'{len(failures)} intended-envelope collisions; first: {failures[0]}')
print('PASS: intended pitch -73..55, outward yaw 0..15 per side, simultaneous jaw opening 0..20',flush=True)
