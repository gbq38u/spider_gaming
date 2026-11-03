"""Compact two-stick radio controller enclosure, millimetres, CadQuery 2.x.

Coordinates: top/front panel is +Z; removable battery lid is at Z=0.
The KY-023 dimensions are a template for the user's pictured clone. Measure
the real boards, switch, LED and battery holder before printing the body.
"""
from pathlib import Path
import cadquery as cq

ROOT = Path(__file__).resolve().parent
STL = ROOT / "print_stl"
STL.mkdir(exist_ok=True)
W = cq.Workplane

H = 40.0
TOP = 2.4
WALL = 2.6
REAR_REBATE = 1.1
LID_GAP = 0.25
STICK_X = (-39.0, 39.0)
STICK_Y = 8.0
PCB_TOP = 27.0
HOLE_DX = 20.3
HOLE_DY = 26.2

# Balanced polygonal gamepad outline: central battery spine, swept grip wings.
# Flat side runs at x=+/-78 accommodate the snap latch and stationary tongue.
OUTLINE = [
    (-78,-19),(-75,-30),(-65,-40),(-53,-45),(-37,-44),(-25,-46),
    (25,-46),(37,-44),(53,-45),(65,-40),(75,-30),(78,-19),
    (78,14),(74,26),(63,37),(52,44),(38,45),(25,43),
    (15,47),(-15,47),(-25,43),(-38,45),(-52,44),(-63,37),
    (-74,26),(-78,14),
]

def profile(inset=0):
    p=W("XY").polyline(OUTLINE).close()
    return p if not inset else p.offset2D(-inset, kind="intersection")

def slab(inset, z, height):
    return profile(inset).extrude(height).translate((0,0,z))

def box(dx,dy,dz,x=0,y=0,z=0):
    return W("XY").box(dx,dy,dz,centered=(True,True,False)).translate((x,y,z))

def cylinder(d, dz, x=0,y=0,z=0):
    return W("XY").circle(d/2).extrude(dz).translate((x,y,z))

def save(name, shape):
    shape=shape.clean()
    solids=shape.solids().vals()
    if len(solids)!=1:
        print(name,"solids",len(solids),[round(s.Volume(),2) for s in solids])
    assert len(solids) == 1, (name,"disconnected")
    printable=shape
    if name in ("controller_body","joystick_fit_coupon"):
        # Show face on the build plate; bosses and walls then grow upward.
        printable=shape.rotate((0,0,0),(1,0,0),180).translate((0,0,H))
    elif name=="controller_snap_lid":
        printable=shape.translate((0,0,-.15))
    cq.exporters.export(printable,str(STL/(name+".stl")),tolerance=.06,angularTolerance=.08)
    b=shape.val().BoundingBox()
    print(name,round(shape.val().Volume(),1),tuple(round(v,2) for v in (b.xlen,b.ylen,b.zlen)))
    return shape

def enclosure():
    shell=slab(0,0,H)
    # Broad bevels soften the faceted silhouette without spoiling the flat
    # internal datum faces or the slide-in rear rebate.
    shell=shell.edges(">Z").chamfer(1.0)
    shell=shell.cut(slab(REAR_REBATE,-.1,2.65))
    shell=shell.cut(slab(WALL,2.45,H-TOP-2.35))

    # Front apertures clear the metal gimbal, including shaft movement. The
    # PCB itself stays inside the shell on four insulating screw bosses.
    for x in STICK_X:
        for sx in (-1,1):
            for sy in (-1,1):
                bx=x+sx*HOLE_DX/2
                by=STICK_Y+sy*HOLE_DY/2
                shell=shell.union(cylinder(6.2,H-TOP-PCB_TOP+.8,bx,by,PCB_TOP))
                # M2.5 self-tapping pilot, inserted from the open rear.
                shell=shell.cut(cylinder(2.1,10.8,bx,by,PCB_TOP-.2))
        shell=shell.cut(cylinder(28.0,H-PCB_TOP+1,x,STICK_Y,PCB_TOP-.1))
        # Shallow annular accent on the show face.
        ring=cylinder(36.4,.55,x,STICK_Y,H-.5).cut(cylinder(34.8,.8,x,STICK_Y,H-.62))
        shell=shell.cut(ring)

    # User's KCD11 ON/OFF: nominal 15x10 panel window. The asymmetric location
    # avoids the joystick PCBs and leaves the central cell bay unobstructed.
    shell=shell.cut(box(15.25,10.25,TOP+.4,62,-15,H-TOP-.2))
    shell=shell.cut(cylinder(5.15,TOP+.4,0,-28,H-TOP-.2))
    shell=shell.cut(cylinder(8.0,.45,0,-28,H-.4))

    # Left stationary keeper first, then the right spring catch. Both are
    # visible through the side windows and can be cleaned after printing.
    shell=shell.cut(box(5.0,9.0,2.5,-76.8,0,4.9))
    shell=shell.cut(box(5.0,12.0,3.0,76.8,0,10.2))
    return save("controller_body",shell)

def lid():
    # Flush back panel sits within the 1.1-mm rebate; 0.25 mm perimeter gap.
    rear=slab(REAR_REBATE+LID_GAP,.15,2.25)
    # An inner rim steers insertion and stiffens the panel, but has a large
    # central opening for the battery and electronics.
    rim=slab(WALL+.28,2.4,3.1).cut(slab(WALL+2.0,2.35,3.3))
    rear=rear.union(rim)

    # Standard single-18650 HOLDER (not the bare cell): 77.1x21.1x21.5 mm.
    # The clear tray is 79x22.6. Raised sides/end stops prevent it from
    # sliding; two cable-tie pairs let users secure holders of varied design.
    for x in (-12.25,12.25):
        rear=rear.union(box(1.9,79,3.2,x,0,2.4))
    for y in (-40.45,40.45):
        rear=rear.union(box(24.5,1.9,3.2,0,y,2.4))
    for y in (-24,24):
        for x in (-16.8,16.8):
            rear=rear.cut(box(2.3,4.5,2.8,x,y,.0))

    # Flat electronics regions either side of the cell; open strap slots
    # accommodate Nano, RF module, regulator and wires with different boards.
    for sign in (-1,1):
        for y in (-17,17):
            for x in (sign*28,sign*52):
                rear=rear.cut(box(2.4,5.0,2.8,x,y,0))

    # Left entry tongue. It is deliberately rigid, with an upward-facing hook.
    rear=rear.union(box(2.0,8.0,4.0,-74.2,0,2.4))
    rear=rear.union(box(1.5,8.0,1.5,-75.25,0,5.1))

    # PETG cantilever at right. Length ~10 mm limits bending strain at snap;
    # two slots isolate its sides from the continuous guiding rim.
    rear=rear.union(box(1.55,10.0,9.7,74.25,0,2.4))
    for y in (-5.6,5.6):
        rear=rear.cut(box(2.3,1.4,9.9,74.0,y,2.35))
    # Small outer hook: press through the side window to release.
    hook=(W("XZ").polyline([(74.675,10.7),(76.025,10.7),
                              (75.15,11.95),(74.675,11.95)]).close()
          .extrude(8.0).translate((0,4.0,0)))
    rear=rear.union(hook)
    # Fingernail recess in the back panel beside the release window.
    rear=rear.cut(box(6.0,9.0,1.2,68.0,-16.0,0))
    return save("controller_snap_lid",rear)

def joystick_coupon():
    # Print this first if the supplied clone has not been measured. Same
    # aperture, screw grid and front-to-PCB axial offset as the main body.
    p=box(46,44,TOP,0,0,H-TOP)
    for sx in (-1,1):
        for sy in (-1,1):
            x=sx*HOLE_DX/2;y=sy*HOLE_DY/2
            p=p.union(cylinder(6.2,H-TOP-PCB_TOP+.8,x,y,PCB_TOP))
            p=p.cut(cylinder(2.1,10.8,x,y,PCB_TOP-.2))
    p=p.cut(cylinder(28.0,H-PCB_TOP+1,0,0,PCB_TOP-.1))
    return save("joystick_fit_coupon",p)

if __name__=="__main__":
    enclosure();lid();joystick_coupon()
