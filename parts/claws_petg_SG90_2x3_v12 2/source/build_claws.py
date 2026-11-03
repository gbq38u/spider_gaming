"""Parametric SG90 metal-gear spider claws, dimensions in millimetres.
Run with CadQuery 2.8: PYTHONPATH=<cadlib> python3 build_claws.py
The two full mechanism STLs are view-only assemblies; individual STLs are for printing.
"""
import math, struct
from pathlib import Path
import cadquery as cq

OUT=Path(__file__).parent
PARTS=OUT/'stl';PARTS.mkdir(exist_ok=True)
P=OUT/'preview';P.mkdir(exist_ok=True)
W=cq.Workplane
# Source servo body is 23 x 12 x 24.2; lug span 32.5; output gearbox center is 5.45 mm off body center.
BODY_X=23.0; BODY_Y=12.0; FIT=0.18; WALL=1.8
GEAR_OFFSET=5.45
# Measured on SPIDER_LIGHT/hanger.STL: single-arm horn 21.673 x 7.33 mm,
# 4 mm hub and 2 mm arm.  The leg STLs use an 8.1 x 3.9 mm bearing seat.
HORN_EDGE=[(0,3.665,3.665),(.107,2.951,4.379),(.420,2.300,5.030),
           (.910,1.769,5.561),(1.534,1.406,5.924),(2.238,1.243,6.087),
           (14.397,3.223,4.107),(14.608,2.365,4.965),
           (15.019,1.583,5.747),(15.605,.922,6.408),
           (16.332,.420,6.910),(17.158,.106,7.224),
           (17.745,.011,7.319),(18.035,0,7.330),
           (18.912,.106,7.224),(19.738,.420,6.910),
           (20.465,.922,6.408),(21.051,1.583,5.747),
           (21.462,2.365,4.965),(21.673,3.223,4.107)]
HORN_OUTLINE=[(18.035-x,lo-3.665) for x,lo,hi in HORN_EDGE]
HORN_OUTLINE += [(18.035-x,hi-3.665) for x,lo,hi in reversed(HORN_EDGE)]
# The horn has two axially staggered arm sections. Their separate projected
# edges create a concave outline that does not enclose all of its own mesh.
# A planar convex envelope with 0.18 mm fit allowance clears both sections.
def convex_hull(points):
    pts=sorted(set(points))
    def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    low=[];high=[]
    for p in pts:
        while len(low)>1 and cross(low[-2],low[-1],p)<=0:low.pop()
        low.append(p)
    for p in reversed(pts):
        while len(high)>1 and cross(high[-2],high[-1],p)<=0:high.pop()
        high.append(p)
    return low[:-1]+high[:-1]
HORN_ENVELOPE=convex_hull(HORN_OUTLINE)
BEARING_R=4.05
FRAME_DX=85.457;FRAME_DY=83.888;FRAME_DZ=93.155
FRAME_TOP=20.5+FRAME_DZ
# Towers now sit above the front hip joints, substantially closer to the
# centreline. The frame still uses its original eight drilling centres.
TOWER_X={-1:115.0,1:241.5}
TOWER_Y=333.0
LIFT=32.0
ANCHOR_X={-1:85.0+FRAME_DX,1:101.0+FRAME_DX}
HOLE_X={-1:(81.0+FRAME_DX,89.0+FRAME_DX),
         1:(97.0+FRAME_DX,105.0+FRAME_DX)}
HOLE_Y=(251.0+FRAME_DY,258.0+FRAME_DY)

def box(dx,dy,dz,x=0,y=0,z=0):
    return W('XY').box(dx,dy,dz,centered=(True,True,False)).translate((x,y,z))
def cyl_z(r,h,x=0,y=0,z=0):
    return W('XY').circle(r).extrude(h).translate((x,y,z))
def cyl_x(r,h,x=0,y=0,z=0):
    return W('YZ').circle(r).extrude(h).translate((x,y,z))
def cut_x(shape,r,h,x,y,z):return shape.cut(cyl_x(r,h,x,y,z))
def cut_z(shape,r,h,x,y,z):return shape.cut(cyl_z(r,h,x,y,z))
def horn_xy(x,y,z,arm_sign,depth=2.2):
    # The arm points along +/-Y; the hub is cleared separately through the part.
    pts=[(cross,arm_sign*along) for along,cross in HORN_ENVELOPE]
    return W('XY').polyline(pts).close().offset2D(.18).extrude(depth).translate((x,y,z))
def horn_yz(x,y,z,x_direction,depth=2.2):
    pts=[(along,cross) for along,cross in HORN_ENVELOPE]
    return W('YZ').polyline(pts).close().offset2D(.18).extrude(x_direction*depth).translate((x,y,z))
def export(name,obj):
    obj=obj.clean()
    cq.exporters.export(obj,str(PARTS/(name+'.stl')),tolerance=0.08,angularTolerance=0.08)
    solids=obj.solids().vals()
    print(name,'solids',len(solids),'volume',round(sum(s.Volume() for s in solids),1),'bbox',tuple(round(v,1) for v in (obj.val().BoundingBox().xlen,obj.val().BoundingBox().ylen,obj.val().BoundingBox().zlen)),flush=True)
    return obj

def servo_socket():
    """Upright insertion pocket. Servo bottom at z=2; mounting ears rest at z=19.2."""
    cx=BODY_X+2*FIT;cy=BODY_Y+2*FIT
    o=box(cx+2*WALL,cy+2*WALL,19.2)
    o=o.cut(box(cx,cy,20,0,0,2.0))
    # Small compliant PETG nibs provide a light press fit in the last 3 mm.
    for yy in (-1,1):
        for xx in (-6.0,6.0):
            o=o.union(box(3.0,0.34,2.0,xx,yy*(cy/2-0.16),15.8))
    # The physical servo lugs are left free. The case is held by the snug
    # socket; the outside horn and opposite bearing retain the joint axially.
    return o.clean()
SOCKET=servo_socket()
coupon=box(27.0,16.0,8.0).cut(box(23.36,12.36,8.2,0,0,1.8))
for yy in (-1,1):
    for xx in (-6.0,6.0):coupon=coupon.union(box(3.0,0.34,2.0,xx,yy*(12.36/2-0.16),4.5))
export('servo_fit_coupon',coupon)
horn_coupon=box(14,30,7,0,8,0)
# A 2 mm shelf supports the long arm; the lower hub alone passes through.
horn_coupon=horn_coupon.cut(horn_xy(0,0,3.8,1,3.3))
horn_coupon=cut_z(horn_coupon,3.82,7.2,0,0,-.1)
export('horn_fit_coupon',horn_coupon)
bearing_coupon=box(16,16,5)
bearing_coupon=cut_z(bearing_coupon,BEARING_R,4.1,0,0,.9)
bearing_coupon=cut_z(bearing_coupon,2.6,1.1,0,0,-.1)
export('bearing_fit_coupon',bearing_coupon)
m3_coupon=box(12,12,9)
m3_coupon=cut_z(m3_coupon,1.35,9.2,0,0,-.1)
export('m3_pilot_fit_coupon',m3_coupon)
reference_bearing=cyl_z(4.0,4.0).cut(cyl_z(1.5,4.2,0,0,-.1))
cq.exporters.export(reference_bearing,str(P/'reference_bearing_3x8x4.stl'),tolerance=.04)
def yaw_base(side):
    # Raised servo deck clears the complete upper leg, including its yaw sweep.
    p=box(34,25,4,0,9,LIFT).edges('|Z').fillet(2.0)
    ax=ANCHOR_X[side]-TOWER_X[side]
    ay=sum(HOLE_Y)/2-TOWER_Y
    p=p.union(box(15.5,23.0,4,ax,ay,0).edges('|Z').fillet(1.0))
    for xx in HOLE_X[side]:
        for yy in HOLE_Y:p=cut_z(p,1.7,4.8,xx-TOWER_X[side],yy-TOWER_Y,-0.5)
    inner=-side
    # Two sturdy posts and short diagonal knees make a light triangulated
    # load path. Their positions leave all four bolt heads accessible.
    for y_root,y_anchor in ((2.0,ay-9.0),(18.0,ay+9.0)):
        p=p.union(box(6.0,3.6,LIFT-2,ax,y_anchor,4))
        x0=inner*14.5
        dx=ax-x0;dy=y_anchor-y_root
        length=math.hypot(dx,dy)
        rail=box(length+4,5.2,7,0,0,LIFT-2)
        rail=rail.rotate((0,0,0),(0,0,1),math.degrees(math.atan2(dy,dx)))
        p=p.union(rail.translate(((ax+x0)/2,(y_anchor+y_root)/2,0)))
        start=cq.Vector(ax,y_anchor,5.0)
        end=cq.Vector(ax-math.copysign(14,ax),y_anchor,LIFT+1)
        vec=end-start
        p=p.union(cq.Workplane(obj=cq.Solid.makeCylinder(2.9,vec.Length,start,vec.normalized())))
    # Raised light servo socket; the ribs touch its outer wall, not the cavity.
    p=p.union(box(27.3,16,4,0,12,LIFT+4))
    s=SOCKET
    if side==-1:s=s.rotate((0,0,0),(0,0,1),180)
    s=s.translate((side*GEAR_OFFSET,12,LIFT+7))
    p=p.union(s)
    p=p.cut(box(5,3.0,4.5,side*GEAR_OFFSET,4.4,LIFT+7)) # cable exit
    # Install the M3 screw from inside the empty servo box, head flush below
    # the servo floor. Its shaft projects down into the bearing in the cradle.
    p=p.union(cyl_z(2.4,1.3,0,12,LIFT-1.3))
    p=cut_z(p,1.35,11.6,0,12,LIFT-1.4)
    p=cut_z(p,2.95,3.2,0,12,LIFT+6.0)
    return p.clean()

def pitch_cradle(side):
    # The horn arm is OUTSIDE the moving plate. Its lower hub reaches the servo
    # shaft through a centre opening while the plate sits under the long arm.
    p=cyl_z(14.0,3.0,0,12,40.6).union(cyl_z(9.5,0.8,0,12,39.8))
    p=p.union(box(8,19,3.8,0,-7.5,39.8)) # rear horn arm and bearing yoke root
    # The supplied hanger has a stepped arm: its distal section is at axial
    # 2..4 mm, not 0..2. A visible, rounded keyhole recess on the outer face
    # matches the access style of spider_leg_L1 while keeping a screw land.
    p=p.union(box(10,24,3.2,0,0,43.4))
    p=p.cut(horn_xy(0,12,39.7,-1,4.3))
    p=p.union(box(8,24,2.0,0,2,39.8)) # load-bearing shelf under horn arm
    p=p.cut(horn_xy(0,12,43.7,-1,3.2)) # open top of the screw channel
    p=cut_z(p,3.82,8,0,12,39.3) # stock servo screw, through raised strip
    keyhole=W('XY').center(0,1.5).slot2D(19,5.4,90).extrude(.9).translate((0,0,45.75))
    p=p.cut(keyhole) # shallow exterior counterbore; 1.75 mm roof remains
    # Shorten the disc before the horizontal servo cavity; leave its rear horn holes.
    p=p.cut(box(100,100,8,0,71.5,39))
    # The back yoke passes outside the fixed yaw socket and foot, then under it.
    p=p.union(box(5,4,45.5,0,-15,-5.3))
    p=p.union(box(5,27,5,0,-1.5,-5.3))
    p=p.union(cyl_z(6.5,5,0,12,-5.3))
    p=cut_z(p,BEARING_R,4.2,0,12,-5.4) # 3x8x4 bearing inserted from below
    p=cut_z(p,2.6,1.2,0,12,-1.4) # boss clearance in inner shoulder
    # Pitch servo shaft faces outward. Its pivot is x=side*20, z=44.
    s=SOCKET
    if side==1:s=s.rotate((0,0,0),(0,0,1),180)
    s=s.rotate((0,0,0),(0,1,0),side*90)
    s=s.translate((-side*10.35,28,49.45))
    p=p.union(s)
    # Two ribs stay outside the servo body and tie the socket to the yaw plate.
    for xx in (-7,7):
        p=p.union(box(3,13,4,xx,14.5,41.2))
    # Compact fixed ear: 0.35 mm from the real servo body's far face.  Two
    # stepped braces join it to the socket without intruding into the body.
    ear_x=-14.7 if side==1 else 8.7
    p=p.union(box(6.2,7,5,-side*11.8,20.5,40))
    p=p.union(box(4,5,5,-side*7,19,40))
    p=p.union(cyl_x(6.3,6.0,ear_x,28,44))
    p=p.union(cyl_x(2.4,1.3,-16 if side==1 else 14.7,28,44))
    # The head enters from the empty pitch-servo box, then its M3 shaft passes
    # through the fixed ear and projects into the moving fork's bearing.
    p=cut_x(p,1.35,8.9,ear_x-1.4 if side==1 else ear_x-.1,28,44)
    p=cut_x(p,2.95,3.4,-11.65 if side==1 else 8.25,28,44)
    return p.clean().translate((0,0,LIFT))

def boom(side):
    # Pitch horn is at x=side*20; the moving beam sits 3 mm beyond it.
    xbase=23 if side==1 else -27
    profile=[(-11,-9),(13,-9),(28,-7),(100,-6),(147,-6),(153,-5),(153,5),(147,6),(100,6),(28,7),(13,9),(-11,9)]
    p=W('YZ').polyline(profile).close().extrude(4).translate((xbase,0,0))
    # The horn arm sits outside a 2 mm wall and clamps that wall toward the
    # servo. Only its lower hub passes through the centre opening.
    drive_face=23 if side==1 else -23
    p=p.union(box(3.4,25,11,side*28.7,8.5,-5.5))
    p=p.cut(horn_yz(drive_face,0,0,side,4.3))
    p=p.union(box(2,24,8,side*24,9,-4)) # moving wall under horn arm
    p=p.cut(horn_yz(side*27,0,0,side,4.0)) # exterior screw access
    p=cut_x(p,3.82,9.2,22 if side==1 else -31.2,0,0)
    outer_slot=W('YZ').center(10.5,0).slot2D(19,5.4,0).extrude(side*1.0)
    p=p.cut(outer_slot.translate((side*29.5,0,0))) # shallow visible keyhole
    for yy,zz,rr in [(135,0,1.7),(147,0,1.7)]:
        p=cut_x(p,rr,6,xbase-1,yy,zz)
    # Rounded lightening windows preserve two continuous rails.
    for yy,ln in [(42,23),(75,25),(109,25)]:
        slot=W('YZ').center(yy,0).slot2D(ln,5.2,0).extrude(6).translate((xbase-1,0,0))
        p=p.cut(slot)
    # A short opposite fork follows the servo body and presses a bearing around
    # the fixed pilot in the cradle.  The crosspiece stays ahead of the servo.
    opposite_x=-19 if side==1 else 15
    op=W('YZ').polyline([(-11,-9),(13,-9),(32,-5),(32,5),(13,9),(-11,9)]).close().extrude(4).translate((opposite_x,0,0))
    p=p.union(op).union(box(46,7,10,side*4,29,-5))
    if side==1:
        p=p.union(cyl_x(6.3,1,-20,0,0))
        p=cut_x(p,BEARING_R,4.2,-20.1,0,0) # open outer insertion side
        p=cut_x(p,2.6,1.2,-16.1,0,0) # fixed boss clears inner shoulder
    else:
        p=p.union(cyl_x(6.3,1,19,0,0))
        p=cut_x(p,BEARING_R,4.2,15.9,0,0)
        p=cut_x(p,2.6,1.2,14.9,0,0)
    return p.clean()

def gripper_holder(side):
    # Mount flange is just outside the boom. 0.35 mm side clearance before bolting.
    xb=27.35 if side==1 else -31.35
    p=box(4.0,32,14,xb+2,137, -7) # y121..153
    for yy in (135,147):p=cut_x(p,1.7,6,xb-1,yy,0)
    # The servo axis is inverted and +42 degrees from boom. At -73 working
    # pitch the tips slope down to the floor while the bearing cap clears it.
    s=SOCKET
    for xx in (-8.5,8.5):
        for yy in (-12.5,12.5):
            if xx==8.5 and yy==-side*12.5:continue
            s=s.union(box(4.6,8.0,5.2,xx,yy/2+4.0*(1 if yy>0 else -1),14.0))
            s=s.union(cyl_z(2.4,5.2,xx,yy,14.0))
            s=cut_z(s,0.85,5.6,xx,yy,13.8)
    # The forward/outboard post would cross the moving horn skirt. Put the
    # third fastener directly behind the servo where the jaw never sweeps.
    s=s.union(box(6,18,5.2,0,side*13,14.0))
    s=s.union(cyl_z(2.4,5.2,0,side*20,14.0))
    s=cut_z(s,0.85,5.6,0,side*20,13.8)
    if side==1:s=s.rotate((0,0,0),(0,0,1),180)
    horn_x=side*GEAR_OFFSET # output after right-hand half-turn; left = -5.45
    s=s.translate((-horn_x,0,-30.35)).rotate((0,0,0),(0,1,0),180).rotate((0,0,0),(1,0,0),42).translate((xb+2+side*9,160,0))
    p=p.union(s)
    # Clearance around the moving boom. The gear bridge is separately removable.
    p=p.cut(box(5.2,44,20,side*25,139,-10))
    return p.clean()

def gear_bridge(side):
    xb=27.35 if side==1 else -31.35
    horn_x=side*GEAR_OFFSET
    body_x=-horn_x
    # Three M2 screws let the servo enter the socket before the bridge is fixed.
    bridge=box(53,33,3,side*10,0,31.1)
    bridge=cut_z(bridge,10.25,4,0,0,30.6)
    # Let the full-size horn skirt pass through the lower support plate.
    bridge=bridge.cut(box(11,24,4,0,12,30.6))
    # A horn fixed to the driven jaw sweeps a sector while opening. Clear its
    # full stock-envelope through the lower plate, including intermediate
    # angles. The sector stays on the opposite side from the passive pivot.
    skirt_outline=[(cross,along) for along,cross in HORN_ENVELOPE]
    skirt=W('XY').polyline(skirt_outline).close().offset2D(2.7)
    vertices=[(v.X,v.Y) for v in skirt.val().Vertices()]
    swept=[]
    for degrees in range(-8,44,2):
        a=math.radians(side*degrees);c=math.cos(a);s=math.sin(a)
        swept.extend((c*x-s*y,s*x+c*y) for x,y in vertices)
    bridge=bridge.cut(W('XY').polyline(convex_hull(swept)).close()
                      .extrude(4.0).translate((0,0,30.7)))
    passive_x=side*20
    bridge=bridge.union(cyl_z(4.0,2.0,passive_x,0,29.1))
    # Fixed Ø4.8 boss reaches only the inner race of a bearing pressed into
    # the passive jaw. The screw threads into this bridge, never clamps the jaw.
    bridge=bridge.union(cyl_z(2.4,2.9,passive_x,0,34.1))
    bridge=cut_z(bridge,1.35,8.2,passive_x,0,29.0) # M3 self-tapping pilot
    for xx in (body_x-8.5,body_x+8.5):
        for yy in (-12.5,12.5):
            if (abs(xx)<5.6 and yy>0) or (xx*side<0 and yy>0):continue
            bridge=bridge.union(cyl_z(2.3,12.0,xx,yy,19.2))
            bridge=cut_z(bridge,1.1,15.5,xx,yy,19.0)
    bridge=bridge.union(box(8,10,3,body_x,-18,31.1))
    bridge=bridge.union(cyl_z(2.3,12.0,body_x,-20,19.2))
    bridge=cut_z(bridge,1.1,15.5,body_x,-20,19.0)
    # Removable upper bearing cap supports the driven jaw opposite its servo.
    # Its lower face is 1 mm above the rotating jaw boss.
    for xx in (-9,9):bridge=bridge.union(cyl_z(2.5,16.2,xx,-17.3,34.1))
    cap=cyl_z(7.5,5,0,0,45.3)
    cap=cap.union(box(12,12,5,0,-10,45.3))
    cap=cap.union(box(22,6,5,0,-14,45.3))
    cap=cut_z(cap,BEARING_R,4.2,0,0,46.2) # bearing inserted from above
    cap=cut_z(cap,2.6,1.2,0,0,45.2) # jaw boss clears inner shoulder
    bridge=bridge.union(cap)
    bridge=bridge.translate((0,0,-30.35)).rotate((0,0,0),(0,1,0),180).rotate((0,0,0),(1,0,0),42).translate((xb+2+side*9,160,0))
    return bridge.clean()

def gear_wire(teeth=14,module=1.42,phase=0):
    rp=module*teeth/2; rb=rp*math.cos(math.radians(20));root=rp-1.25*module;tip=rp+module
    ap=math.tan(math.radians(20))-math.radians(20)
    ht=math.pi/(2*teeth)
    pts=[]
    for k in range(teeth):
        c=2*math.pi*k/teeth+phase
        for sign in (-1,1):
            radii=[root,rb]+[rb+(tip-rb)*j/5 for j in range(1,6)]
            if sign==1:radii=list(reversed(radii))
            for r in radii:
                inv=0 if r<=rb else math.tan(math.acos(rb/r))-math.acos(rb/r)
                th=c+sign*(ht+ap-inv)
                pts.append((r*math.cos(th),r*math.sin(th)))

    return pts

def jaw(driven,side):
    # 20.0 mm pivot spacing and module 1.42 give 0.12 mm centre-distance backlash.
    gear=W('XY').polyline(gear_wire(phase=0 if driven else math.pi/14)).close().extrude(4)
    # Curved inward tip; mirrored finger on passive gear.
    sign=(side if driven else -side)
    finger=[(-3,4),(-6,10),(-8,35),(-6,46),(5,55),(9,53),(9,50),(0,42),(-2,34),(0,13),(3,7)]
    finger=[(sign*x,y) for x,y in finger]
    f=W('XY').polyline(finger).close().extrude(4.2)
    p=gear.union(f)
    if driven:
        # The gear is fitted only AFTER the stock horn is screwed to the
        # separate carrier. This keeps the carrier between servo and horn.
        for xx in (-4.5,4.5):
            p=cut_z(p,1.1,4.2,xx,4.0,-.1) # M2 clearance into carrier
        # The opposite bearing seats against this integral nub.
        p=p.union(cyl_z(4.5,6.2,0,0,4.0))
        p=p.union(cyl_z(2.4,1.0,0,0,10.2))
        p=cut_z(p,1.35,7.3,0,0,4.0)
    else:
        # The passive jaw rotates freely on its own 3x8x4 bearing. Its outer
        # race is a press fit in the jaw; the fixed bridge boss supports the
        # inner race through the Ø5.2 shoulder clearance.
        p=p.union(cyl_z(6.0,2.2,0,0,3.8))
        p=cut_z(p,BEARING_R,4.2,0,0,1.9)
        p=cut_z(p,2.6,2.1,0,0,-.1)
    # Opposed small grip pads are intentionally replaceable with rubber tape.
    return p.clean()

def jaw_carrier():
    """Removable stock-horn clamp under the driven gear; install before gear."""
    outer=[(cross,along) for along,cross in HORN_ENVELOPE]
    p=W('XY').polyline(outer).close().offset2D(2.0).extrude(4).translate((0,0,-4))
    # A continuous circular root carries both gear screws within its outline.
    # The former pair of projecting round ears added no useful joint function.
    p=p.union(cyl_z(8.5,4,0,0,-4))
    p=p.cut(horn_xy(0,0,-2.0,1,2.2)) # exposed outside arm
    p=cut_z(p,3.82,4.2,0,0,-4.1) # lower hub reaches servo spline
    for xx in (-4.5,4.5):
        p=cut_z(p,0.85,4.2,xx,4.0,-4.1) # M2 self-tapping pilots
    return p.clean()

parts={}
for side,name in [(-1,'left'),(1,'right')]:
    parts[f'yaw_base_{name}']=export(f'yaw_base_{name}',yaw_base(side))
    arm=-side # pitch axis and long arm face the open centre, away from the leg
    parts[f'pitch_cradle_{name}']=export(f'pitch_cradle_{name}',pitch_cradle(arm))
    parts[f'boom_{name}']=export(f'boom_{name}',boom(arm))
    parts[f'gripper_holder_{name}']=export(f'gripper_holder_{name}',gripper_holder(arm))
    parts[f'gear_bridge_{name}']=export(f'gear_bridge_{name}',gear_bridge(arm))
for side,name in [(-1,'left'),(1,'right')]:
    parts[f'jaw_driven_{name}']=export(f'jaw_driven_{name}',jaw(True,-side))
    parts[f'jaw_carrier_{name}']=export(f'jaw_carrier_{name}',jaw_carrier())
    parts[f'jaw_passive_{name}']=export(f'jaw_passive_{name}',jaw(False,-side))
# Drill guides duplicate each base footprint and bolt grid.
for side,name in [(-1,'left'),(1,'right')]:
    jig=box(15.5,23.0,2)
    for xx in (-4,4):
        for yy in (-3.5,3.5):jig=cut_z(jig,1.6,3,xx,yy,-0.5)
    export(f'drill_jig_{name}',jig)
print('done',flush=True)
