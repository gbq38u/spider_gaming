#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <stdbool.h>

#include "structs.h"
#include "kinematics.h"

/* Generate the swing phase of a single step along the X axis.
 *
 * Fills `points[0..num_of_steps]` with num_of_steps+1 (pos, vel) samples.
 * `pos` is Cartesian displacement from `start.pos`. `vel` is Cartesian
 * velocity in units/second.
 */
void X_swing_traj(int num_of_steps, point_t * points, const point_t start, const float swing_time, const float x_speed);

/* Generate the stance phase of a single step along the X axis.
 *
 * Fills `points[0..num_of_steps]` with num_of_steps+1 (pos, vel) samples.
 * `pos` is Cartesian displacement from `start.pos`. `vel` is Cartesian
 * velocity in units/second.
 */
void X_stance_traj(int num_of_steps, point_t * points, const point_t start, const point_t end, const float stance_time);

/* Generate the swing phase of a single step along the Y axis.
 *
 * Fills `points[0..num_of_steps]` with num_of_steps+1 (pos, vel) samples.
 * `pos` is Cartesian displacement from `start.pos`. `vel` is Cartesian
 * velocity in units/second.
 */
void Y_swing_traj(int num_of_steps, point_t * points, const point_t start, const float swing_time, const float y_speed);

/* Generate the stance phase of a single step along the Y axis.
 *
 * Fills `points[0..num_of_steps]` with num_of_steps+1 (pos, vel) samples.
 * `pos` is Cartesian displacement from `start.pos`. `vel` is Cartesian
 * velocity in units/second.
 */
void Y_stance_traj(int num_of_steps, point_t * points, const point_t start, const point_t end, const float stance_time);

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
                const float x_speed,          const float y_speed, 
                const int num_of_swing_steps, const int num_of_stance_steps);

#endif //TRAJECTORY_H


