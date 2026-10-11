#ifndef ESPNODE_H
#define ESPNODE_H

#include <math.h>

#define RAD2DEG (180.0f / (float)M_PI)

static inline float constrainf(float x, float lo, float hi)
{
    return fminf(fmaxf(x, lo), hi);
}

// converts angles to degrees and writes them into servo_angles
void set_servo_angles(float * servo_angles, float a1, float a2, float a3);

#endif //ESPNODE_H
