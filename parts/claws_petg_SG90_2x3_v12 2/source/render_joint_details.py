"""Close views of the three joints and accessible fastener faces."""
import numpy as np
from assemble import assemble,T
from render_preview import draw_view, P

items=[a for a in assemble(-73) if a[2].endswith('_left')]
by_name={label:(src,m,label) for src,m,label in items}

def show(filename,keys,eye,centre,span,title):
    draw_view(P/filename,[by_name[k+'_left'] for k in keys],eye,centre,
              width=1300,height=1000,title=title,span_mm=span)

show('yaw_joint_detail.png',
     ['yaw_base','pitch_cradle','servo_yaw','horn_yaw','bearing_yaw'],
     (1,-1,.65),np.array([115,345,185.455]),80,
     'YAW  |  stock horn and opposite 3x8x4 bearing')
show('yaw_inside_screw_detail.png',
     ['yaw_base'],
     (0,-.35,1),np.array([115,345,154]),52,
     'YAW  |  M3 head recess inside empty servo socket')
show('pitch_drive_detail.png',
     ['pitch_cradle','boom','servo_pitch','horn_pitch','bearing_pitch'],
     (1,-.7,.5),np.array([115,361,189.655]),75,
     'PITCH  |  horn-side screw access')
show('pitch_bearing_detail.png',
     ['pitch_cradle','boom','servo_pitch','horn_pitch','bearing_pitch'],
     (-1,-.7,.4),np.array([115,361,189.655]),75,
     'PITCH  |  opposite bearing and M3 pilot')
show('pitch_bearing_socket_detail.png',
     ['pitch_cradle','boom','servo_pitch'],
     (-1,-.7,.4),np.array([115,361,189.655]),55,
     'PITCH  |  open outer bearing seat close to servo case')
show('pitch_inside_screw_detail.png',
     ['pitch_cradle'],
     (1,-.25,.2),np.array([115,361,189.655]),55,
     'PITCH  |  M3 head recess inside empty servo socket')
jaw_matrix=by_name['jaw_driven_left'][1]
grip_centre=(jaw_matrix@np.array([0,0,2,1]))[:3]
grip_eye=jaw_matrix[:3,:3]@np.array([.45,-.4,1])
show('grip_joint_detail.png',
     ['gripper_holder','gear_bridge','jaw_carrier','jaw_driven','jaw_passive',
      'servo_grip','horn_grip','bearing_grip','bearing_passive_jaw'],
     grip_eye,grip_centre,75,
     'GRIP  |  two bearing-supported jaws, stock horn')
show('grip_horn_clamp_detail.png',
     ['gripper_holder','servo_grip','jaw_carrier','horn_grip'],
     grip_eye,grip_centre,52,
     'GRIP CLAMP  |  stock horn outside printed carrier')
gear_src,gear_pose,_=by_name['jaw_driven_left']
draw_view(P/'grip_horn_clamp_exploded.png',
          [by_name[k+'_left'] for k in ('gripper_holder','servo_grip','jaw_carrier','horn_grip')]
          +[(gear_src,gear_pose@T(38,0,12),'jaw_driven_left')],
          grip_eye,grip_centre,width=1300,height=1000,span_mm=100,
          title='GRIP CLAMP  |  horn outside carrier, gear added with two M2')
show('grip_bearing_pilot_detail.png',
     ['gear_bridge','jaw_carrier','jaw_driven'],
     grip_eye,grip_centre,50,
     'GRIP  |  bearing seat and integral M3 pilot, no loose spacer')
passive_centre=(jaw_matrix@np.array([20,0,2,1]))[:3]
show('passive_jaw_bearing_detail.png',
     ['gear_bridge','jaw_passive','bearing_passive_jaw'],
     grip_eye,passive_centre,50,
     'PASSIVE JAW  |  outer race in jaw, fixed M3 axle in bridge')
