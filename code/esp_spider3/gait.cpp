#include "gait.h"
#include "trajectory.h"

#include <math.h>
#include <esp_timer.h>

/* Duty cycle: 50/50 (tripod). Must sum to <= STDNUMOFSTEPS. */
#define SWING_STEPS   2
#define STANCE_STEPS  2

/* ---------- persistent state ---------- */

static cycle_t main_cycle;
static cycle_t inertial_cycle;

static int     current_step   = 0;
static int64_t step_start_us  = 0;

static float         cmd_x_speed    = 0.0f;
static float         cmd_y_speed    = 0.0f;
static volatile bool cmd_dirty = false;

static float   servo_angles_rad[3] = { 0.0f, 0.0f, 0.0f };

/* ---------- helpers ---------- */

static inline float eval_poly(const float c[4], float t)
{
    /* c[0]·t³ + c[1]·t² + c[2]·t + c[3], Horner form */
    return ((c[0]*t + c[1])*t + c[2])*t + c[3];
}

// /* Forward kinematics: joint angles (rad) → Cartesian foot pos (cm).
//  * Matches the Python FK convention: X forward (-cos), Y left, Z up. */
// static void leg_fk(const leg_t * leg, const float q[3], float xyz[3])
// {
//     const float L1 = leg->LinkLengths[0];
//     const float L2 = leg->LinkLengths[1];
//     const float L3 = leg->LinkLengths[2];
// 
//     const float phi1 = q[0] + leg->JointOffsets[0];
//     const float phi2 = q[1] + leg->JointOffsets[1];
//     const float phi3 = q[2] + leg->JointOffsets[2];
// 
//     const float S1  = sinf(phi1), C1 = cosf(phi1);
//     const float S2  = sinf(phi2);
//     const float ph23 = phi3 - phi2;
//     const float S23 = sinf(ph23), C23 = cosf(ph23);
// 
//     const float R = L1 + L2*S2 + L3*S23;
// 
//     xyz[0] = -R * C1;
//     xyz[1] =  R * S1;
//     xyz[2] =  L2 * cosf(phi2) - L3 * C23;
// }

/* Regenerate the inertial (reference) cycle, anchored
 * at the fixed start pos, with the current commanded speeds. */
static void regenerate_inertial(const leg_t * leg)
{
    float xyz[3];

    xyz[0] =   0.0 - (float)(SWING_STEPS) * cmd_x_speed / 2;
    xyz[1] =  18.6 - (float)(SWING_STEPS) * cmd_y_speed / 2;
    xyz[2] = -10.0;

    point_t start;
    start.pos[0]  = xyz[0];
    start.pos[1]  = xyz[1];
    start.pos[2]  = xyz[2];
    start.vels[0] = 0.0f;
    start.vels[1] = 0.0f;
    start.vels[2] = 0.0f;

    step_cycle(&inertial_cycle, leg, start, cmd_x_speed, cmd_y_speed, SWING_STEPS, STANCE_STEPS);
}

/* ---------- public API ---------- */

void gait_init(const leg_t * leg)
{
    /* Seed servo angle tracking from the leg's home pose. */
    servo_angles_rad[0] = leg->JointHome[0];
    servo_angles_rad[1] = leg->JointHome[1];
    servo_angles_rad[2] = leg->JointHome[2];

    cmd_x_speed = 0.0f;
    cmd_y_speed = 0.0f;
    cmd_dirty   = false;

    /* Generate the initial (stationary) inertial cycle. */
    regenerate_inertial(leg);

    /* Seed the main cycle fully from the reference. */
    main_cycle.NumOfSteps = inertial_cycle.NumOfSteps;
    for (int i = 0; i < main_cycle.NumOfSteps; i++) {
        main_cycle.Steps[i] = inertial_cycle.Steps[i];
    }

    current_step  = 0;
    step_start_us = esp_timer_get_time();
}

void gait_set_command(float x_speed, float y_speed)
{
    cmd_x_speed = x_speed;
    cmd_y_speed = y_speed;
    cmd_dirty   = true;
}

static void advance_step(void)
{
    const int N = main_cycle.NumOfSteps;

    step_t * prev = &main_cycle.Steps[current_step];

    current_step  = (current_step + 1) % N;

    step_t * curr = &main_cycle.Steps[current_step];
    
    step_start_us = esp_timer_get_time();

    /* Start where the previous step ended. This makes the trajectory
     * continuous. Normally a no-op, but it fixes any mismatch left
     * over from a previous advance that changed prev->end. */
    curr->start = prev->end;

    /* Take the reference endpoint for this step from the inertial
     * cycle. Because the inertial cycle is regenerated from the
     * current pose on every command change, this pulls the leg
     * toward the reference within one step. */
    curr->end = inertial_cycle.Steps[current_step].end;

    /* Recompute the polynomial for the freshly-updated step. */
    update_step_coefficients(curr);
}

void gait_motion_tick(const leg_t * leg)
{
    if (cmd_dirty) {
        regenerate_inertial(leg);
        cmd_dirty = false;
    }

    const int64_t now_us     = esp_timer_get_time();
    const int64_t elapsed_us = now_us - step_start_us;
    const int64_t step_us    = (int64_t)(STDSTEPTIME * 1e6f);

    if (elapsed_us >= step_us) {
        advance_step();
    }
}

void gait_servo_tick()
{
    const step_t * s = &main_cycle.Steps[current_step];

    float t = (float)(esp_timer_get_time() - step_start_us) / 1e6f;
    if (t < 0.0f)         { t = 0.0f; }
    if (t > s->StepTime)  { t = s->StepTime; }

    for (int j = 0; j < 3; j++) {
        servo_angles_rad[j] = eval_poly(s->coefficients[j], t);
    }
}

const float * gait_get_current_angles()
{
    return servo_angles_rad;
}