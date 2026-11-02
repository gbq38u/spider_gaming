#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <stdbool.h>

#include "structs.h"
#include "kinematics.h"

/* Swing: quartic Bezier with edge velocities matched to stance velocity.
 * Stance: linear, constant velocity −x_speed (or −y_speed).
 * Both generate num_of_steps+1 samples (pos, vel) in leg-local Cartesian. */

/* Generate the swing phase of a single step along the X axis.
 *
 * Fills `points[0..num_of_steps]` with num_of_steps+1 (pos, vel) samples.
 * `pos` is Cartesian displacement from `start.pos`. `vel` is Cartesian
 * velocity in units/second.
 */
void X_swing_traj(int num_of_steps, point_t * points, const point_t start, const float height, const float x_speed, const float swing_time, const float stance_time);

/* Generate the stance phase of a single step along the X axis.
 *
 * Fills `points[0..num_of_steps]` with num_of_steps+1 (pos, vel) samples.
 * `pos` is Cartesian displacement from `start.pos`. `vel` is Cartesian
 * velocity in units/second.
 */
void X_stance_traj(int num_of_steps, point_t * points, const point_t start, const float x_speed, const float stance_time);

/* Generate the swing phase of a single step along the Y axis.
 *
 * Fills `points[0..num_of_steps]` with num_of_steps+1 (pos, vel) samples.
 * `pos` is Cartesian displacement from `start.pos`. `vel` is Cartesian
 * velocity in units/second.
 */
void Y_swing_traj(int num_of_steps, point_t * points, const point_t start, const float height, const float y_speed, const float swing_time, const float stance_time);

/* Generate the stance phase of a single step along the Y axis.
 *
 * Fills `points[0..num_of_steps]` with num_of_steps+1 (pos, vel) samples.
 * `pos` is Cartesian displacement from `start.pos`. `vel` is Cartesian
 * velocity in units/second.
 */
void Y_stance_traj(int num_of_steps, point_t * points, const point_t start, const float y_speed, const float stance_time);

void X_step_cycle_traj(int num_swing, int num_stance, const float step_time, point_t * points, const point_t start, const float height, const float x_speed);

void Y_step_cycle_traj(int num_swing, int num_stance, const float step_time, point_t * points, const point_t start, const float height, const float y_speed);

/* Generate one full XY step cycle:
 *
 *   1. Cartesian X swing+stance trajectory
 *   2. Cartesian Y swing+stance trajectory
 *   3. Sum the two trajectories in Cartesian space
 *   4. IK on the summed points  (Cartesian pos → joint angles)
 *   5. Jacobian                 (Cartesian vel → joint angular vel)
 *   6. Distribute into cycle->Steps[]
 *
 * Coefficients are NOT computed here.
 *
 * Buffer requirement: cycle->Steps sized for >= num_of_steps entries.
 */
void step_cycle(cycle_t * cycle, const leg_t * leg, const point_t start,
                const float height,
                const float x_speed, const float y_speed,
                const int num_of_swing_steps, const int num_of_stance_steps);



#endif //TRAJECTORY_H


