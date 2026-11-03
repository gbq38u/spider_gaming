"""Geometric regression checks for the three servo/bearing joints (mm)."""
import runpy
import numpy as np
import cadquery as cq
from assemble import T, Rx, Ry, Rz, read_stl, SRC, HORN_LOCAL, HORN_YAW, HORN_GRIP

build = runpy.run_path('build_claws.py')
part = build['parts']
bearing = build['reference_bearing']
checks = 0

def ok(condition, message):
    global checks
    if not condition:
        raise AssertionError(message)
    checks += 1

def clear(shape, point, message):
    ok(not shape.val().isInside(cq.Vector(*point), 1e-6), message)

def area(path):
    a = read_stl(path).reshape(-1, 3)
    return a.min(axis=0), a.max(axis=0)

# Source measurements, not generic SG90 catalogue dimensions.  The supplied
# servo/horn meshes omit their actual shaft spline and fastener holes.
lo, hi = area(SRC/'hanger.STL')
ok(abs((hi-lo)[0]-21.673)<.02, 'source horn length changed')
ok(abs((hi-lo)[2]-7.33)<.02, 'source horn width changed')
lo, hi = area(SRC/'servo.STL')
ok(abs((hi-lo)[0]-32.5)<.02, 'source servo lug span changed')
# The box follows the leg-photo construction: four shallow friction nibs,
# without printed pads or pilots under the servo's own mounting tabs.
socket=build['SOCKET']
for xx in (-14.25,14.25):
    clear(socket,(xx,0,17.5),'printed servo lug pad has returned')
for xx in (-6.0,6.0):
    for yy in (-5.92,5.92):
        ok(socket.val().isInside(cq.Vector(xx,yy,16.5),1e-6),
           'servo retention nib missing')
        clear(socket,(xx,yy*5.6/5.92,16.5),
              'servo cavity obstructed beyond the intended nib')
triangles = read_stl(SRC/'hanger.STL')
horn_samples = np.unique(np.concatenate((triangles.reshape(-1,3), triangles.mean(axis=1))).round(5),axis=0)

for name, arm in [('left', 1), ('right', -1)]:
    base = part['yaw_base_'+name]
    cradle = part['pitch_cradle_'+name]
    boom = part['boom_'+name]
    jaw = part['jaw_driven_'+name]
    carrier = part['jaw_carrier_'+name]
    passive_jaw = part['jaw_passive_'+name]
    bridge = part['gear_bridge_'+name]
    for xx in (-7.6,7.6):
        clear(carrier,(xx,4.0,-3.0),f'{name}: projecting jaw-carrier ear returned')

    # Test the actual supplied triangulated horn, including its elevated long
    # arm, against every printed drive part. The source STL has no screw holes.
    pitch_horn = np.array([[0,0,arm,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]],float)
    for shape, matrix, joint in (
        (cradle,T(0,12,71.8)@HORN_YAW@HORN_LOCAL,'yaw'),
        (boom,T(arm*23)@pitch_horn@HORN_LOCAL,'pitch'),
        (carrier,T(0,0,-4)@HORN_GRIP@HORN_LOCAL,'grip')):
        points = horn_samples@matrix[:3,:3].T+matrix[:3,3]
        # Native axial y=2 is the intended contact plane: the outside horn
        # arm presses the printed shelf there. No stock-horn volume may enter
        # the shelf or surrounding wall on either side of that plane.
        inside = sum(shape.val().isInside(cq.Vector(*q),1e-7)
                     for native,q in zip(horn_samples,points)
                     if abs(native[1]-2.0)>1e-4)
        ok(inside==0, f'{name}: supplied {joint} horn penetrates printed part at {inside} mesh samples')

    for u in (7.0,12.0,17.0):
        ok(cradle.val().isInside(cq.Vector(0,12-u,73.2),1e-6),
           f'{name}: yaw horn has no clamping shelf at {u} mm')
        clear(cradle,(0,12-u,74.2),f'{name}: yaw horn outer face blocked at {u} mm')
        ok(boom.val().isInside(cq.Vector(arm*24,u,0),1e-6),
           f'{name}: pitch horn has no clamping shelf at {u} mm')
        clear(boom,(arm*26,u,0),f'{name}: pitch horn outer face blocked at {u} mm')
        ok(carrier.val().isInside(cq.Vector(0,u,-3),1e-6),
           f'{name}: grip horn has no clamping shelf at {u} mm')
        clear(carrier,(0,u,-1),f'{name}: grip horn outer face blocked at {u} mm')

    # The yaw and pitch M3 screws enter through the empty servo cavities. Their
    # head counterbores have room for a 5.5 x 3.0 mm socket head and finish
    # below the corresponding servo body after installation.
    for z in np.linspace(30.65,41.1,16):
        clear(base,(0,12,z),f'{name}: inside-out yaw M3 bore blocked at {z}')
    for z in (38.2,39.5,41.05):
        clear(base,(2.75,12,z),f'{name}: yaw head counterbore blocked at {z}')
    ok(base.val().isInside(cq.Vector(2.75,12,37.8),1e-6),
       f'{name}: yaw M3 head has no flush seating shoulder')
    for x in np.linspace(-16.0,-8.3,16) if arm==1 else np.linspace(8.3,15.9,16):
        clear(cradle,(x,28,76),f'{name}: inside-out pitch M3 bore blocked at {x}')
    for x in (-11.4,-10,-8.35) if arm==1 else (8.35,10,11.4):
        clear(cradle,(x,30.75,76),f'{name}: pitch head counterbore blocked at {x}')
    seat_x=-11.8 if arm==1 else 11.8
    ok(cradle.val().isInside(cq.Vector(seat_x,30.75,76),1e-6),
       f'{name}: pitch M3 head has no flush seating shoulder')

    # Test the bores through each rotating member, including bearing shoulders.
    for z in np.linspace(26.65, 31.65, 12):
        clear(cradle, (0, 12, z), f'{name}: yaw bearing M3 lip blocked at {z}')
    for z in np.linspace(30.75, 39.05, 12):
        clear(base, (0, 12, z), f'{name}: yaw fixed M3 pilot blocked at {z}')
    bearing_x = -20 if arm == 1 else 16
    ear_x = -14.7 if arm == 1 else 8.7
    pitch_outer = (-20.05,-14.95) if arm==1 else (15.05,20.05)
    for x in np.linspace(*pitch_outer, 12):
        clear(boom, (x, 0, 0), f'{name}: pitch bearing M3 lip blocked at {x}')
    pilot_limits=(ear_x-1.25,ear_x+5.95) if arm==1 else (ear_x+.05,ear_x+7.25)
    for x in np.linspace(*pilot_limits, 12):
        clear(cradle, (x, 28, 76), f'{name}: pitch fixed M3 pilot blocked at {x}')

    # The grip horn arm presses on a printed wall; its screws are accessed
    # through the outer opening above it rather than through that wall.
    for u in (5.5, 12, 17.0):
        clear(cradle, (0, 12-u, 76.0), f'{name}: yaw horn arm screw inaccessible, u={u}')
        clear(boom, (arm*29, u, 0), f'{name}: pitch horn arm screw inaccessible, u={u}')
        clear(carrier, (0, u, -1), f'{name}: grip horn arm screw inaccessible, u={u}')
    clear(cradle,(2,0,45.9+32),f'{name}: yaw rounded horn recess missing')
    ok(cradle.val().isInside(cq.Vector(3.4,0,45.9+32),1e-6),
       f'{name}: yaw horn recess lost its screw-bearing land')
    clear(boom,(arm*30,10,2),f'{name}: pitch rounded horn recess missing')
    ok(boom.val().isInside(cq.Vector(arm*30,10,3.5),1e-6),
       f'{name}: pitch horn recess lost its screw-bearing land')
    for z in np.linspace(-3.9, -.1, 12):
        clear(carrier, (0, 0, z), f'{name}: gripper horn hub passage blocked at {z}')
    for z in np.linspace(4.2,11.15,12):
        clear(jaw, (0, 0, z), f'{name}: driven jaw M3 pilot blocked at {z}')
    ok(carrier.intersect(jaw).val().Volume()<1e-5,
       f'{name}: carrier and driven gear overlap')
    for xx in (-4.5,4.5):
        for z in np.linspace(-3.9,-.1,8):
            clear(carrier,(xx,4.0,z),f'{name}: carrier M2 pilot blocked')
        for z in np.linspace(.1,3.9,8):
            clear(jaw,(xx,4.0,z),f'{name}: gear M2 clearance blocked')
    for z in np.linspace(0,4.2,10):
        clear(passive_jaw,(0,0,z),
              f'{name}: passive jaw M3 screw path blocked at {z}')
    # The driven jaw's integral nub, with no loose spacer, meets the inner
    # bearing race at holder-local z=15.95.
    ok(abs((4.75+jaw.val().BoundingBox().zmax)-15.95)<.01,
       f'{name}: driven bearing axial stack mismatch')
    ok(abs((32-1.3)-(-5.3+32+4))<1e-6,
       f'{name}: yaw bearing inner race has no fixed support')
    ok(abs((ear_x if arm == 1 else ear_x+6)-
           (bearing_x+4 if arm == 1 else bearing_x))<1.31,
       f'{name}: pitch bearing inner race offset too large')

    # A nominal 3x8x4 annulus must sit freely in every cup and meet its fixed
    # support without overlapping either printed part.
    yaw_ring=bearing.translate((0,12,26.7))
    ok(yaw_ring.intersect(cradle).val().Volume()<1e-5,
       f'{name}: yaw bearing intersects cup')
    ok(yaw_ring.intersect(base).val().Volume()<1e-5,
       f'{name}: yaw bearing intersects fixed deck')
    pitch_ring=bearing.rotate((0,0,0),(0,1,0),90).translate((bearing_x,0,0))
    ok(pitch_ring.intersect(boom).val().Volume()<1e-5,
       f'{name}: pitch bearing intersects cup')
    ok(pitch_ring.translate((0,28,76)).intersect(cradle).val().Volume()<1e-5,
       f'{name}: pitch bearing intersects fixed ear')
    grip_x=(27.35 if arm==1 else -31.35)+2+arm*9
    grip_ring=(bearing.translate((0,0,15.95))
               .rotate((0,0,0),(0,1,0),180)
               .rotate((0,0,0),(1,0,0),42)
               .translate((grip_x,160,0)))
    ok(grip_ring.intersect(bridge).val().Volume()<1e-5,
       f'{name}: grip bearing intersects upper cap')
    grip_nub=(jaw.translate((0,0,4.75))
              .rotate((0,0,0),(0,1,0),180)
              .rotate((0,0,0),(1,0,0),42)
              .translate((grip_x,160,0)))
    ok(grip_ring.intersect(grip_nub).val().Volume()<1e-5,
       f'{name}: integral grip nub penetrates bearing')
    ok(bridge.intersect(grip_nub).val().Volume()<1e-5,
       f'{name}: fixed grip bridge collides with driven jaw')
    carrier_pose=(carrier.translate((0,0,4.75))
                  .rotate((0,0,0),(0,1,0),180)
                  .rotate((0,0,0),(1,0,0),42)
                  .translate((grip_x,160,0)))
    ok(bridge.intersect(carrier_pose).val().Volume()<1e-5,
       f'{name}: fixed grip bridge collides with horn carrier')
    passive_pose=(passive_jaw.translate((arm*20,0,4.75))
                  .rotate((0,0,0),(0,1,0),180)
                  .rotate((0,0,0),(1,0,0),42)
                  .translate((grip_x,160,0)))
    ok(bridge.intersect(passive_pose).val().Volume()<1e-5,
       f'{name}: fixed grip bridge collides with passive jaw')
    passive_ring=(bearing.translate((arm*20,0,6.65))
                  .rotate((0,0,0),(0,1,0),180)
                  .rotate((0,0,0),(1,0,0),42)
                  .translate((grip_x,160,0)))
    ok(passive_ring.intersect(passive_pose).val().Volume()<1e-5,
       f'{name}: passive bearing intersects press-fit cup')
    ok(passive_ring.intersect(bridge).val().Volume()<1e-5,
       f'{name}: passive bearing intersects fixed bridge boss')
    ok(abs((4.75+1.9)-(37.0-30.35))<1e-6,
       f'{name}: passive bearing inner ring misses fixed bridge boss')

    # The compact ear ends 0.35 mm outside the simplified servo body. Test a
    # conservative core of the pitch-servo insertion space.
    ok(abs((ear_x+6 if arm==1 else ear_x)-(-8.35 if arm==1 else 8.35))>.34,
       f'{name}: compact ear is too close to servo case')
    for x in (-7,-3,3,7):
        for y in (23,28,33):
            for z in (73,77,85):
                clear(cradle,(arm*x,y,z),
                      f'{name}: pitch servo insertion space blocked')

    # Adjacent printed pieces must clear through the documented pitch poses.
    ok(base.intersect(cradle).val().Volume()<1e-5,
       f'{name}: fixed base collides with yaw cradle')
    for angle in (-73, 0, 20, 55):
        moving = boom.rotate((0,0,0),(1,0,0),angle).translate((0,28,76))
        ok(cradle.intersect(moving).val().Volume()<1e-5,
           f'{name}: pitch fork collides at {angle} degrees')

    # The holder and removable bridge use precisely the same three M2 axes.
    horn_x = arm*5.45
    body_x = -horn_x
    xb = 27.35 if arm == 1 else -31.35
    grip_x = xb+2+arm*9
    orient = T(grip_x,160)@Rx(42)@Ry(180)
    for z in np.linspace(14.95,19.95,10):
        q=orient@np.array([0,0,z,1])
        clear(bridge,q[:3],f'{name}: driven-grip bearing M3 path blocked at {z}')
    for z in np.linspace(-1.2,6.6,16):
        q=orient@np.array([arm*20,0,z,1])
        clear(bridge,q[:3],f'{name}: passive pivot M3 pilot blocked at {z}')
    holder_matrix = orient@T(-horn_x,0,-30.35)@(Rz(180) if arm == 1 else np.eye(4))
    bridge_matrix = orient@T(0,0,-30.35)
    holder_axes = [holder_matrix@np.array([xx,yy,17,1])
                   for xx in (-8.5,8.5) for yy in (-12.5,12.5)
                   if not (xx==8.5 and yy==-arm*12.5)]
    holder_axes.append(holder_matrix@np.array([0,arm*20,17,1]))
    bridge_axes = [bridge_matrix@np.array([xx,yy,25,1])
                   for xx in (body_x-8.5,body_x+8.5) for yy in (-12.5,12.5)
                   if not ((abs(xx)<5.6 and yy>0) or (xx*arm<0 and yy>0))]
    bridge_axes.append(bridge_matrix@np.array([body_x,-20,25,1]))
    axis = orient[:3,2]
    for q in bridge_axes:
        ok(min(np.linalg.norm(np.cross(q[:3]-p[:3],axis)) for p in holder_axes)<1e-7,
           f'{name}: bridge M2 post misses holder pilot')
    # Both boom flange M3 axes match the holder flange axes (y=135,147,z=0).
    for y in (135,147):
        clear(boom, (arm*25,y,0), f'{name}: boom flange M3 hole blocked')
        clear(part['gripper_holder_'+name],
              (29.35 if arm==1 else -29.35,y,0),
              f'{name}: holder flange M3 hole blocked')

for label, shape in part.items():
    ok(len(shape.solids().vals()) == 1, f'{label}: disconnected print body')

print(f'PASS: {checks} geometry assertions; source horn/servo measured; '
      'M3 axes, horn access, M2/M3 joint centres and four pitch poses checked.')
