#ifndef GAIT_H
#define GAIT_H

#include <stdbool.h>

#include "structs.h"
#include "kinematics.h"

#define SWING_STEPS   5
#define STANCE_STEPS  5

/* Initialize a single leg's cycle state. Call once per leg after the
 * leg struct is populated and the servos are ready. `start_step` sets
 * the leg's phase offset within the shared reference cycle:
 *   0 for the first tripod group
 *   half_cycle for the second tripod group
 *   any other offset for wave / ripple gaits. */
void gait_init(leg_t * leg, int start_step);

/* New robot-wide speed command. Sets a flag; the actual regeneration
 * happens in the next gait_motion_tick call. */
void gait_set_command(float x_speed, float y_speed);

/* Advance one leg's cycle state machine. Call from the motion timer
 * for every leg. */
void gait_motion_tick(leg_t * leg);

/* Evaluate the current step's polynomial for one leg and write the
 * resulting joint angles into leg->Servos.ServoAngles (degrees). */
void gait_servo_tick(leg_t * leg);

void gait_regenerate_all(spider_t * spider);

bool gait_consume_cmd_dirty(void);

#endif // GAIT_H