#include "trajectory.h"

#include <math.h>
#include <stdbool.h>

/* ---- easing ---- */

static inline float ease_in_out(float t)
{
    float tt = t * t;
    return 3.0f * tt - 2.0f * tt * t;          /* 3t² − 2t³ */
}

static inline float ease_in_out_d(float t)
{
    return 6.0f * t - 6.0f * t * t;            /* 6t − 6t² */
}

/* ---- quartic Bezier ---- */

static void bezier_curve(float t, const float p0[3], const float p1[3], const float p2[3], const float p3[3], const float p4[3], float out[3])
{
    const float u    = 1.0f - t;
    const float uu   = u * u;
    const float uuu  = uu * u;
    const float uuuu = uuu * u;
    const float tt   = t * t;
    const float ttt  = tt * t;
    const float tttt = ttt * t;

    const float b0 = uuuu;
    const float b1 = 4.0f * uuu * t;
    const float b2 = 6.0f * uu  * tt;
    const float b3 = 4.0f * u   * ttt;
    const float b4 = tttt;

    for (int i = 0; i < 3; i++) {
        out[i] = b0 * p0[i] + b1 * p1[i] + b2 * p2[i] + b3 * p3[i] + b4 * p4[i];
    }
}

static void bezier_curve_d(float t, const float p0[3], const float p1[3], const float p2[3], const float p3[3], const float p4[3], float out[3])
{
    const float u   = 1.0f - t;
    const float uu  = u * u;
    const float uuu = uu * u;
    const float tt  = t * t;
    const float ttt = tt * t;

    const float c1 =  4.0f * uuu;      /*  4·(1-t)³   */
    const float c2 = 12.0f * uu * t;   /* 12·(1-t)²·t */
    const float c3 = 12.0f * u  * tt;  /* 12·(1-t)·t² */
    const float c4 =  4.0f * ttt;      /*  4·t³       */

    for (int i = 0; i < 3; i++) {
        out[i] = c1 * (p1[i] - p0[i]) + c2 * (p2[i] - p1[i]) + c3 * (p3[i] - p2[i]) + c4 * (p4[i] - p3[i]);
    }
}

/* ---- swing shape ----
 *
 * `axis` is 0 for X motion, 1 for Y motion. The stroke is along that
 * axis, the lift is always on Z (index 2).
 *
 * `k` enforces edge velocities to match stance velocity for any duty:
 *     v(0) = v(1) = −l / stance_time = −x_speed (or −y_speed)
 *
 * Peak Z during the swing is 0.5·h. When X and Y are superposed, the
 * combined lift is 0.5·h_X + 0.5·h_Y.
 */
static void swing_func(const float t, const float l, const float h, const float swing_time, const float stance_time, const int axis, float out[3], float out_d[3])
{
    const float k = swing_time / (4.0f * stance_time); // the coefficient necessary for keeping right derivatives in the edges of swing trajectory

    float p0[3] = { 0.0f, 0.0f, 0.0f };
    float p1[3] = { 0.0f, 0.0f, 0.0f };
    float p2[3] = { 0.0f, 0.0f, 0.0f };
    float p3[3] = { 0.0f, 0.0f, 0.0f };
    float p4[3] = { 0.0f, 0.0f, 0.0f };

    p1[axis] =     -k * l;
    p2[axis] =          l;
    p2[2]    = 1.333f * h;      // with such p2 the leg will lift up to 0.5 * h
    p3[axis] = (1.0f + k) * l;
    p4[axis] =          l;

    bezier_curve  (t, p0, p1, p2, p3, p4, out);
    bezier_curve_d(t, p0, p1, p2, p3, p4, out_d);
}

/* ---- per-axis swing ---- */

void X_swing_traj(int num_of_steps, point_t * points, const point_t start, const float height, const float x_speed, const float swing_time, const float stance_time)
{
    if (num_of_steps < 1 || swing_time <= 0.0f || stance_time <= 0.0f) { return; }

    const float len            = stance_time * x_speed;
    const int   n_points       = num_of_steps + 1;
    const float inv_num_steps  = 1.0f / (float)num_of_steps;
    const float inv_swing_time = 1.0f / swing_time;

    for (int i = 0; i < n_points; i++) {
        const float t = (float)i * inv_num_steps; /* in [0,1] */

        float pos[3], dpos[3];
        swing_func(t, len, height, swing_time, stance_time, 0, pos, dpos);

        points[i].pos[0]  = pos[0]  + start.pos[0];
        points[i].pos[1]  = pos[1]  + start.pos[1];
        points[i].pos[2]  = pos[2]  + start.pos[2];

        points[i].vels[0] = dpos[0] * inv_swing_time;
        points[i].vels[1] = dpos[1] * inv_swing_time;
        points[i].vels[2] = dpos[2] * inv_swing_time;
    }
}

void Y_swing_traj(int num_of_steps, point_t * points, const point_t start, const float height, const float y_speed, const float swing_time, const float stance_time)
{
    if (num_of_steps < 1 || swing_time <= 0.0f || stance_time <= 0.0f) { return; }

    const float len            = stance_time * y_speed;
    const int   n_points       = num_of_steps + 1;
    const float inv_num_steps  = 1.0f / (float)num_of_steps;
    const float inv_swing_time = 1.0f / swing_time;

    for (int i = 0; i < n_points; i++) {
        const float t = (float)i * inv_num_steps;

        float pos[3], dpos[3];
        swing_func(t, len, height, swing_time, stance_time, 1, pos, dpos);

        points[i].pos[0]  = pos[0]  + start.pos[0];
        points[i].pos[1]  = pos[1]  + start.pos[1];
        points[i].pos[2]  = pos[2]  + start.pos[2];

        points[i].vels[0] = dpos[0] * inv_swing_time;
        points[i].vels[1] = dpos[1] * inv_swing_time;
        points[i].vels[2] = dpos[2] * inv_swing_time;
    }
}

/* ---- per-axis stance (linear, constant velocity) ---- */

void X_stance_traj(int num_of_steps, point_t * points, const point_t start, const float x_speed, const float stance_time)
{
    if (num_of_steps < 1 || stance_time <= 0.0f) { return; }

    const float start_x       = start.pos[0];
    const float dx            = -x_speed * stance_time;
    const int   n_points      = num_of_steps + 1;
    const float inv_num_steps = 1.0f / (float)num_of_steps;

    for (int i = 0; i < n_points; i++) {
        const float t = (float)i * inv_num_steps;

        points[i].pos[0]  = start_x + dx * t;
        points[i].pos[1]  = start.pos[1];
        points[i].pos[2]  = start.pos[2];

        points[i].vels[0] = -x_speed;
        points[i].vels[1] =  0.0f;
        points[i].vels[2] =  0.0f;
    }
}

void Y_stance_traj(int num_of_steps, point_t * points, const point_t start, const float y_speed, const float stance_time)
{
    if (num_of_steps < 1 || stance_time <= 0.0f) { return; }

    const float start_y       = start.pos[1];
    const float dy            = -y_speed * stance_time;
    const int   n_points      = num_of_steps + 1;
    const float inv_num_steps = 1.0f / (float)num_of_steps;

    for (int i = 0; i < n_points; i++) {
        const float t = (float)i * inv_num_steps;

        points[i].pos[0]  = start.pos[0];
        points[i].pos[1]  = start_y + dy * t;
        points[i].pos[2]  = start.pos[2];

        points[i].vels[0] =  0.0f;
        points[i].vels[1] = -y_speed;
        points[i].vels[2] =  0.0f;
    }
}

/* ---- per-axis full cycle: swing + stance ---- */

void X_step_cycle_traj(int num_swing, int num_stance, const float step_time, point_t * points, const point_t start, const float height, const float x_speed)
{
    const float swing_time  = (float)num_swing  * step_time;
    const float stance_time = (float)num_stance * step_time;

    X_swing_traj(num_swing, points, start, height, x_speed, swing_time, stance_time);

    const point_t swing_end = points[num_swing];
    X_stance_traj(num_stance, points + num_swing, swing_end, x_speed, stance_time);
}

void Y_step_cycle_traj(int num_swing, int num_stance, const float step_time, point_t * points, const point_t start, const float height, const float y_speed)
{
    const float swing_time  = (float)num_swing  * step_time;
    const float stance_time = (float)num_stance * step_time;

    Y_swing_traj(num_swing, points, start, height, y_speed, swing_time, stance_time);

    const point_t swing_end = points[num_swing];
    Y_stance_traj(num_stance, points + num_swing, swing_end, y_speed, stance_time);
}

/* ---- full XY cycle: generate, superpose, IK, pack ---- */

void step_cycle(cycle_t * cycle, const leg_t * leg, const point_t start,
                const float height,
                const float x_speed, const float y_speed,
                const int num_of_swing_steps, const int num_of_stance_steps)
{
    if (cycle == NULL || leg == NULL)          { return; }
    if (num_of_swing_steps  < 1)               { return; }
    if (num_of_stance_steps < 1)               { return; }

    const int num_of_steps = num_of_swing_steps + num_of_stance_steps;
    if (num_of_steps > STDNUMOFSTEPS)          { return; }

    const int n_points = num_of_steps + 1;

    point_t x_pts[STDNUMOFSTEPS + 1] = {0};
    point_t y_pts[STDNUMOFSTEPS + 1] = {0};

    /* 1 & 2. Per-axis trajectories -------------------------------- */
    X_step_cycle_traj(num_of_swing_steps, num_of_stance_steps, STDSTEPTIME, x_pts, start, height, x_speed);

    Y_step_cycle_traj(num_of_swing_steps, num_of_stance_steps, STDSTEPTIME, y_pts, start, height, y_speed);

    /* 3. Cartesian superposition ---------------------------------- */
    /*
     * XY: sum of the two axes minus start (start is baked into both).
     * Z : both axes contribute 0.5·h during swing, so summing gives
     *     up to h total lift on a diagonal. During stance both
     *     contribute zero lift, so the sum reduces to start.pos[2].
     */
    for (int i = 0; i < n_points; i++) {
        x_pts[i].pos[0]  = x_pts[i].pos[0]  + y_pts[i].pos[0]  - start.pos[0];
        x_pts[i].pos[1]  = x_pts[i].pos[1]  + y_pts[i].pos[1]  - start.pos[1];
        x_pts[i].pos[2]  = x_pts[i].pos[2]  + y_pts[i].pos[2]  - start.pos[2];

        x_pts[i].vels[0] = x_pts[i].vels[0] + y_pts[i].vels[0];
        x_pts[i].vels[1] = x_pts[i].vels[1] + y_pts[i].vels[1];
        x_pts[i].vels[2] = x_pts[i].vels[2] + y_pts[i].vels[2];
    }

    /* 4 & 5. IK + Jacobian ---------------------------------------- */
    doIK      (leg, n_points, x_pts);
    doJacobian(leg, n_points, x_pts);

    /* 6. Pack into the cycle -------------------------------------- */
    cycle_init(cycle, num_of_steps, x_pts, STDSTEPTIME);
}