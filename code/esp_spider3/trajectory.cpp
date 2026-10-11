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

/* ---- cubic Bezier and its derivative ----
 *
 * Control points are (3,)-arrays. t is a scalar in [0,1].
 * Output is written into out[3]. No dynamic allocation.
 */
static void bezier_curve(float t, const float p0[3], const float p1[3], const float p2[3], const float p3[3], float out[3])
{
    const float u   = 1.0f - t;
    const float uu  = u * u;
    const float uuu = uu * u;
    const float tt  = t * t;
    const float ttt = tt * t;

    const float b0  = uuu;
    const float b1  = 3.0f * uu * t;
    const float b2  = 3.0f * u  * tt;
    const float b3  = ttt;

    for (int i = 0; i < 3; i++) {
        out[i] = b0 * p0[i] + b1 * p1[i] + b2 * p2[i] + b3 * p3[i];
    }
}

/* dB(u) / du */
static void bezier_curve_d(float t, const float p0[3], const float p1[3], const float p2[3], const float p3[3], float out[3])
{
    const float u  = 1.0f - t;
    const float c1 = 3.0f * u * u;             /* 3(1-t)² */
    const float c2 = 6.0f * u * t;             /* 6(1-t)t */
    const float c3 = 3.0f * t * t;             /* 3t²     */

    for (int i = 0; i < 3; i++) {
        out[i] = c1 * (p1[i] - p0[i]) + c2 * (p2[i] - p1[i]) + c3 * (p3[i] - p2[i]);
    }
}

void X_swing_traj(int num_of_steps, point_t * points, const point_t start, const float swing_time, const float x_speed)
{
    /* Sanity: at least one step, or we can't parameterize t */
    if (num_of_steps < 1 || swing_time <= 0.0f) { return; }

    const float len     = swing_time * x_speed;
    const float abs_len = fabsf(len);

    const float p0[3] = { 0.0f,  0.0f,       0.0f      };
    const float p1[3] = { 0.0f,  0.0f, 0.3f * abs_len  };
    const float p2[3] = { len,   0.0f, 0.3f * abs_len  };
    const float p3[3] = { len,   0.0f,       0.0f      };

    const int n_points = num_of_steps + 1;

    const float inv_num_steps = 1.0f / (float)num_of_steps;
    const float inv_swing_time = 1.0f / swing_time;

    for (int i = 0; i < n_points; i++) 
    {
        const float t  = (float)i * inv_num_steps;      /* in [0,1] */
        const float e  = ease_in_out(t);
        const float ed = ease_in_out_d(t);

        float pos[3];
        float dpos[3];
        bezier_curve  (e, p0, p1, p2, p3, pos);
        bezier_curve_d(e, p0, p1, p2, p3, dpos);

        /* Position: offset by start, same as `points += starting_point` */
        points[i].pos[0] = pos[0] + start.pos[0];
        points[i].pos[1] = pos[1] + start.pos[1];
        points[i].pos[2] = pos[2] + start.pos[2];

        /* Velocity: chain rule.  t_real = t * swing_time, so
         * du/dt_real = ed / swing_time. */
        points[i].vels[0] = dpos[0] * ed * inv_swing_time;
        points[i].vels[1] = dpos[1] * ed * inv_swing_time;
        points[i].vels[2] = dpos[2] * ed * inv_swing_time;
    }
}

void X_stance_traj(int num_of_steps, point_t * points, const point_t start, const point_t end, const float stance_time)
{
  /* Sanity: at least one step, or we can't parameterize t */
  if (num_of_steps < 1 || stance_time <= 0.0f) { return; }

  const float start_x = start.pos[0];
  const float end_x   = end.pos[0];

  const int n_points = num_of_steps + 1;

  const float inv_num_of_steps   = 1.0f / (float)num_of_steps;
  const float inv_stance_time = 1.0f / stance_time;

  for (int i = 0; i < n_points; i++)
  {
    const float t = (float)i * inv_num_of_steps;
    const float e  = ease_in_out(t);
    const float ed = ease_in_out_d(t);

    float pos  = start_x + (end_x - start_x) * e;
    float dpos =           (end_x - start_x) * ed;

    points[i].pos[0]  = pos;
    points[i].pos[1]  = start.pos[1];
    points[i].pos[2]  = start.pos[2];

    points[i].vels[0] = dpos * inv_stance_time;
    points[i].vels[1] = 0.0f;
    points[i].vels[2] = 0.0f;
  }

}

void Y_swing_traj(int num_of_steps, point_t * points, const point_t start, const float swing_time, const float y_speed)
{
    /* Sanity: at least one step, or we can't parameterize t */
    if (num_of_steps < 1 || swing_time <= 0.0f) { return; }

    const float len     = swing_time * y_speed;
    const float abs_len = fabsf(len);

    const float p0[3] = { 0.0f,  0.0f,       0.0f      };
    const float p1[3] = { 0.0f,  0.0f, 0.3f * abs_len  };
    const float p2[3] = { 0.0f,   len, 0.3f * abs_len  };
    const float p3[3] = { 0.0f,   len,       0.0f      };

    const int n_points = num_of_steps + 1;

    const float inv_num_steps = 1.0f / (float)num_of_steps;
    const float inv_swing_time = 1.0f / swing_time;

    for (int i = 0; i < n_points; i++) 
    {
        const float t  = (float)i * inv_num_steps;      /* in [0,1] */
        const float e  = ease_in_out(t);
        const float ed = ease_in_out_d(t);

        float pos[3];
        float dpos[3];
        bezier_curve  (e, p0, p1, p2, p3, pos);
        bezier_curve_d(e, p0, p1, p2, p3, dpos);

        /* Position: offset by start, same as `points += starting_point` */
        points[i].pos[0] = pos[0] + start.pos[0];
        points[i].pos[1] = pos[1] + start.pos[1];
        points[i].pos[2] = pos[2] + start.pos[2];

        /* Velocity: chain rule.  t_real = t * swing_time, so
         * du/dt_real = ed / swing_time. */
        points[i].vels[0] = dpos[0] * ed * inv_swing_time;
        points[i].vels[1] = dpos[1] * ed * inv_swing_time;
        points[i].vels[2] = dpos[2] * ed * inv_swing_time;
    }
}

void Y_stance_traj(int num_of_steps, point_t * points, const point_t start, const point_t end, const float stance_time)
{
  /* Sanity: at least one step, or we can't parameterize t */
  if (num_of_steps < 1 || stance_time <= 0.0f) { return; }

  const float start_y = start.pos[1];
  const float end_y   = end.pos[1];

  const int n_points = num_of_steps + 1;

  const float inv_num_of_steps   = 1.0f / (float)num_of_steps;
  const float inv_stance_time = 1.0f / stance_time;

  for (int i = 0; i < n_points; i++)
  {
    const float t = (float)i * inv_num_of_steps;
    const float e  = ease_in_out(t);
    const float ed = ease_in_out_d(t);

    float pos  = start_y + (end_y - start_y) * e;
    float dpos =           (end_y - start_y) * ed;

    points[i].pos[0]  = start.pos[0];
    points[i].pos[1]  = pos;
    points[i].pos[2]  = start.pos[2];

    points[i].vels[0] = 0.0f; 
    points[i].vels[1] = dpos * inv_stance_time;
    points[i].vels[2] = 0.0f;
  }
}

void X_step_cycle_traj(int num_swing, int num_stance, const float step_time, point_t * points, const point_t start, const float x_speed)
{
    const float swing_time  = (float)num_swing  * step_time;
    const float stance_time = (float)num_stance * step_time;

    X_swing_traj(num_swing, points, start, swing_time, x_speed);

    const point_t swing_end = points[num_swing];
    X_stance_traj(num_stance, points + num_swing, swing_end, start, stance_time);
}

void Y_step_cycle_traj(int num_swing, int num_stance, const float step_time, point_t * points, const point_t start, const float y_speed)
{
    const float swing_time  = (float)num_swing  * step_time;
    const float stance_time = (float)num_stance * step_time;

    Y_swing_traj(num_swing, points, start, swing_time, y_speed);

    const point_t swing_end = points[num_swing];
    Y_stance_traj(num_stance, points + num_swing, swing_end, start, stance_time);
}

void step_cycle(cycle_t * cycle, const leg_t * leg, const point_t start,
                const float x_speed,          const float y_speed,
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

    /* 1 & 2. Per-axis Cartesian trajectories ---------------------- */
    X_step_cycle_traj(num_of_swing_steps, num_of_stance_steps, STDSTEPTIME, x_pts, start, x_speed);

    Y_step_cycle_traj(num_of_swing_steps, num_of_stance_steps, STDSTEPTIME, y_pts, start, y_speed);

    /* 3. Cartesian superposition ---------------------------------- */
    for (int i = 0; i < n_points; i++) 
    {
        x_pts[i].pos[0] =  x_pts[i].pos[0] + y_pts[i].pos[0] - start.pos[0];
        x_pts[i].pos[1] =  x_pts[i].pos[1] + y_pts[i].pos[1] - start.pos[1];

        x_pts[i].vels[0] =  x_pts[i].vels[0] + y_pts[i].vels[0];
        x_pts[i].vels[1] =  x_pts[i].vels[1] + y_pts[i].vels[1];

        if (x_pts[i].pos[2] >= y_pts[i].pos[2]) // kludgy way not to lift the leg too high because of doubling the lifts of two motions
        {
            // x_pts[i].pos[2]  = x_pts[i].pos[2];
            // x_pts[i].vels[2] = x_pts[i].vels[2]; 
        } else
        {
            x_pts[i].pos[2]  = y_pts[i].pos[2]; 
            x_pts[i].vels[2] = y_pts[i].vels[2];
        }

        
    }

    /* 4 & 5. IK + Jacobian --------------------------------------- */
    doIK      (leg, n_points, x_pts);
    doJacobian(leg, n_points, x_pts);

    /* 6. Pack into the cycle -------------------------------------- */
    cycle_init(cycle, num_of_steps, x_pts, STDSTEPTIME);
}






