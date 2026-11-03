#ifndef ESPNODE_H
#define ESPNODE_H

#include <math.h>
#include "structs.h"

#define RAD2DEG (180.0f / (float)M_PI)

static inline float constrainf(float x, float lo, float hi)
{
    return fminf(fmaxf(x, lo), hi);
}

/* Writes joint angles (radians) into leg->Servos.ServoAngles (degrees). */
void leg_set_servo_angles(leg_t * leg, float a1, float a2, float a3);

/* Writes every leg's home pose into leg->Servos.ServoAngles. */
void spider_set_home(spider_t * spider);

#endif // ESPNODE_H
