#ifndef GAIT_H
#define GAIT_H

#include "structs.h"
#include "kinematics.h"

/* Initialize the gait state machine. Call once after the leg struct
 * and servos are ready, before the ROS entities come up. */
void gait_init(const leg_t * leg);

/* New speed command from the PC (or wherever). Thread-safe-ish: just
 * sets a flag, the actual regeneration happens in gait_motion_tick. */
void gait_set_command(float x_speed, float y_speed);

/* Advance the gait state machine. Call from a low-rate timer (~20 ms).
 * Handles command changes and step transitions. */
void gait_motion_tick(const leg_t * leg);

/* Evaluate the current step's polynomial and update the internal
 * servo angle targets. Call from the servo-rate timer (~20 ms). */
void gait_servo_tick(void);

/* Read the current joint angle targets (radians). */
const float * gait_get_current_angles(void);

#endif // GAIT_H