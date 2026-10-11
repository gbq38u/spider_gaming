#include "esp_node.h"

void set_servo_angles(float * servo_angles, float a1, float a2, float a3)
{
  servo_angles[0] = (constrainf(a1 * RAD2DEG, 0.0f, 180.0f));
  servo_angles[1] = (constrainf(a2 * RAD2DEG, 0.0f, 180.0f));
  servo_angles[2] = (constrainf(a3 * RAD2DEG, 0.0f, 180.0f));
}