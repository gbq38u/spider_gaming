#include "gait.h"
#include "trajectory.h"
#include "esp_node.h"

#include <math.h>
#include <esp_timer.h>

/* ---------- robot-wide command state (shared by all legs) ---------- */

static float         cmd_x_speed = 0.0f;
static float         cmd_y_speed = 0.0f;
static volatile bool cmd_dirty   = false;
static bool          cmd_start_pending = false;

/* Distances and speeds are in the same units as LinkLengths (centimetres).
 * Limit the stride to a conservative excursion around the home pose. */
static const float MAX_SWING_LENGTH = 2.0f;

/* ---------- helpers ---------- */

static inline float eval_poly(const float c[4], float t)
{
    return ((c[0]*t + c[1])*t + c[2])*t + c[3];
}

static void regenerate_inertial(leg_t * leg, float x_speed)
{
    float xyz[3] = {
        leg->Geometry.EFHome[0] - (float)SWING_STEPS * x_speed * STDSTEPTIME / 2.0f,
        leg->Geometry.EFHome[1] - (float)SWING_STEPS * cmd_y_speed * STDSTEPTIME / 2.0f,
        leg->Geometry.EFHome[2]
    };

    point_t start;
    start.pos[0]  = xyz[0];
    start.pos[1]  = xyz[1];
    start.pos[2]  = xyz[2];
    start.vels[0] = 0.0f;
    start.vels[1] = 0.0f;
    start.vels[2] = 0.0f;

    step_cycle(&leg->Cycle.Inertial, leg, start, x_speed, cmd_y_speed, SWING_STEPS, STANCE_STEPS);
}

/* Join the steady tripod cycle from the present pose.  At rest every foot
 * is at EFHome, while the reference cycle begins half a stride behind or
 * ahead of it.  Starting directly at the next reference endpoint would
 * send a stance foot forwards on the first step. */
static void start_walking(leg_t * leg, float x_speed)
{
    const int N = leg->Cycle.Cycle.NumOfSteps;
    const int phase = leg->Cycle.StartStepInd;
    cycle_t startup = {};
    if (!startup_half_cycle(&startup, leg, x_speed, cmd_y_speed,
                            phase < SWING_STEPS, SWING_STEPS)) return;
    const int64_t now_us = esp_timer_get_time();
    const step_t * old = &leg->Cycle.Cycle.Steps[leg->Cycle.CurrStepInd];
    float t = (float)(now_us - leg->Cycle.StepStartTimeUS) / 1e6f;
    if (t < 0.0f) t = 0.0f;
    if (t > old->StepTime) t = old->StepTime;

    point_t current = {};
    for (int j = 0; j < 3; j++) {
        current.pos[j] = eval_poly(old->coefficients[j], t);
        current.vels[j] = 0.0f;
    }

    for (int k = 0; k < SWING_STEPS; k++) {
        const int index = (phase + k) % N;
        leg->Cycle.Cycle.Steps[index].end = startup.Steps[k].end;
    }

    step_t * first = &leg->Cycle.Cycle.Steps[phase];
    first->start = current;
    update_step_coefficients(first);
    leg->Cycle.CurrStepInd = phase;
    leg->Cycle.StepStartTimeUS = now_us;
    leg->Cycle.StartupStepsRemaining = SWING_STEPS - 1;
}

/* ---------- public API ---------- */

void gait_init(leg_t * leg, int start_step)
{
    /* Servo tracking starts at the leg's home pose. */
    for (int j = 0; j < 3; j++) {
        leg->Servos.ServoAngles[j] =
            constrainf(leg->Geometry.JointHome[j] * RAD2DEG, 0.0f, 180.0f);
    }

    /* Generate the shared reference if it hasn't been generated yet.
     * Regenerating here (per leg) is idempotent for symmetric legs. */
    if (leg->Cycle.Inertial.NumOfSteps == 0) {
        regenerate_inertial(leg, cmd_x_speed);
    }

    /* Seed this leg's main cycle from the reference. */
    leg->Cycle.Cycle          = leg->Cycle.Inertial;
    leg->Cycle.StartStepInd   = start_step % leg->Cycle.Inertial.NumOfSteps;
    leg->Cycle.CurrStepInd    = leg->Cycle.StartStepInd;
    leg->Cycle.StartupStepsRemaining = 0;
    leg->Cycle.StepStartTimeUS = esp_timer_get_time();
}

void gait_regenerate_all(spider_t * spider)
{
    /* LF RF LM RM LR RR: the two rear legs have the opposite measured
     * longitudinal servo response.  Correct that response before the
     * common body-to-leg rotation; leave every driver/channel unchanged. */
    static const float forward_sign[6] = { 1.0f, 1.0f, 1.0f, 1.0f, -1.0f, -1.0f };
    for (int i = 0; i < 6; i++) {
        regenerate_inertial(spider->Legs[i], forward_sign[i] * cmd_x_speed);
    }
    for (int i = 0; i < 6; i++) {
        if (cmd_start_pending) {
            start_walking(spider->Legs[i], forward_sign[i] * cmd_x_speed);
        } else {
            spider->Legs[i]->Cycle.StartupStepsRemaining = 0;
        }
    }
    cmd_start_pending = false;
}

bool gait_consume_cmd_dirty(void)
{
    if (!cmd_dirty) return false;
    cmd_dirty = false;
    return true;
}

void gait_set_command(float x_speed, float y_speed)
{
    const bool was_stopped = (cmd_x_speed == 0.0f && cmd_y_speed == 0.0f);
    const float swing_time = (float)SWING_STEPS * STDSTEPTIME;
    const float speed_limit = MAX_SWING_LENGTH / swing_time;
    const float speed = hypotf(x_speed, y_speed);
    if (speed > speed_limit) {
        const float scale = speed_limit / speed;
        x_speed *= scale;
        y_speed *= scale;
    }

    cmd_x_speed = x_speed;
    cmd_y_speed = y_speed;
    if (speed == 0.0f) cmd_start_pending = false;
    else if (was_stopped) cmd_start_pending = true;
    cmd_dirty   = true;
}

static void advance_step(leg_t * leg, int64_t step_us)
{
    const int N = leg->Cycle.Cycle.NumOfSteps;
    const int i = leg->Cycle.CurrStepInd;
    const int next = (i + 1) % N;

    step_t * prev = &leg->Cycle.Cycle.Steps[i];
    step_t * curr = &leg->Cycle.Cycle.Steps[next];

    leg->Cycle.CurrStepInd     = next;
    leg->Cycle.StepStartTimeUS += step_us;

    curr->start = prev->end;
    if (leg->Cycle.StartupStepsRemaining > 0) {
        leg->Cycle.StartupStepsRemaining--;
    } else {
        curr->end = leg->Cycle.Inertial.Steps[next].end;
    }
    update_step_coefficients(curr);
}

void gait_motion_tick(leg_t * leg)
{
    const int64_t now_us     = esp_timer_get_time();
    const int64_t step_us    = (int64_t)(STDSTEPTIME * 1e6f);

    if (step_us <= 0) return;
    const int64_t cycle_us = step_us * leg->Cycle.Cycle.NumOfSteps;
    if (cycle_us <= 0) return;
    const int64_t elapsed_us = now_us - leg->Cycle.StepStartTimeUS;
    if (elapsed_us >= cycle_us) {
        leg->Cycle.StepStartTimeUS += (elapsed_us / cycle_us) * cycle_us;
    }
    while (now_us - leg->Cycle.StepStartTimeUS >= step_us) {
        advance_step(leg, step_us);
    }
}

void gait_servo_tick(leg_t * leg)
{
    const step_t * s = &leg->Cycle.Cycle.Steps[leg->Cycle.CurrStepInd];

    float t = (float)(esp_timer_get_time() - leg->Cycle.StepStartTimeUS) / 1e6f;
    if (t < 0.0f)        t = 0.0f;
    if (t > s->StepTime) t = s->StepTime;

    for (int j = 0; j < 3; j++) {
        const float q_rad = eval_poly(s->coefficients[j], t);
        leg->Servos.ServoAngles[j] = constrainf(q_rad * RAD2DEG, 0.0f, 180.0f);
    }
}
