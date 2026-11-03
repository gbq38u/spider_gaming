#include "esp_node.h"

void leg_set_servo_angles(leg_t * leg, float a1, float a2, float a3)
{
    leg->Servos.ServoAngles[0] = constrainf(a1 * RAD2DEG, 0.0f, 180.0f);
    leg->Servos.ServoAngles[1] = constrainf(a2 * RAD2DEG, 0.0f, 180.0f);
    leg->Servos.ServoAngles[2] = constrainf(a3 * RAD2DEG, 0.0f, 180.0f);
}

void spider_set_home(spider_t * spider)
{
    for (int i = 0; i < 6; i++) {
        leg_t * leg = spider->Legs[i];
        leg->Servos.ServoAngles[0] = constrainf(leg->Geometry.JointHome[0] * RAD2DEG, 0.0f, 180.0f);
        leg->Servos.ServoAngles[1] = constrainf(leg->Geometry.JointHome[1] * RAD2DEG, 0.0f, 180.0f);
        leg->Servos.ServoAngles[2] = constrainf(leg->Geometry.JointHome[2] * RAD2DEG, 0.0f, 180.0f);
    }
}