#include "gait.h"
#include "trajectory.h"
#include "esp_node.h"

#include <math.h>
#include <esp_timer.h>

/* ---------- robot-wide command state (shared by all legs) ---------- */

static float         cmd_x_speed = 0.0f;
static float         cmd_y_speed = 0.0f;
static volatile bool cmd_dirty   = false;

/* ---------- helpers ---------- */

static inline float eval_poly(const float c[4], float t)
{
    return ((c[0]*t + c[1])*t + c[2])*t + c[3];
}

static void regenerate_inertial(leg_t * leg)
{
    const float yaw = leg->Geometry.BaseRotOffsets[2];
    const float c = cosf(yaw);
    const float s = sinf(yaw);

    const float x_speed =      cmd_x_speed * c + -1 * cmd_y_speed * s;
    const float y_speed =      cmd_x_speed * s +      cmd_y_speed * c;

    const float l_x = (float)STANCE_STEPS * STDSTEPTIME * x_speed;
    const float l_y = (float)STANCE_STEPS * STDSTEPTIME * y_speed;

    float xyz[3] = {
        leg->Geometry.EFHome[0] - l_x / 2.0f,
        leg->Geometry.EFHome[1] - l_y / 2.0f,
        leg->Geometry.EFHome[2]
    };

    point_t start;
    start.pos[0]  = xyz[0];
    start.pos[1]  = xyz[1];
    start.pos[2]  = xyz[2];
    start.vels[0] = 0.0f;
    start.vels[1] = 0.0f;
    start.vels[2] = 0.0f;

    const float height = 0.5f * sqrt(l_x*l_x + l_y*l_y);

    step_cycle(&leg->Cycle.Inertial, leg, start, height, x_speed, y_speed, SWING_STEPS, STANCE_STEPS);
}

/* ---------- public API ---------- */

void gait_set_command(float x_speed, float y_speed)
{
    cmd_x_speed = x_speed; // not sure about -1
    cmd_y_speed = y_speed; //
    cmd_dirty   = true;
}

void gait_regenerate_all(spider_t * spider)
{
    for (int i = 0; i < 6; i++) {
        regenerate_inertial(spider->Legs[i]);
    }
}

bool gait_consume_cmd_dirty(void)
{
    if (!cmd_dirty) return false;
    cmd_dirty = false;
    return true;
}

void gait_init(leg_t * leg, int start_step)
{
    point_t q;
    for (int j = 0; j < 3; j++) {
        q.pos[j]  = leg->Geometry.EFHome[j];
        q.vels[j] = 0.0f;
    }
    doIK(leg, 1, &q);

    /* Servo tracking starts at the leg's home pose. */
    for (int j = 0; j < 3; j++) {
        leg->Servos.ServoAngles[j] = constrainf(q.pos[j] * RAD2DEG, 0.0f, 180.0f);
    }

    regenerate_inertial(leg);

    /* Seed this leg's main cycle from the reference. */
    leg->Cycle.Cycle           = leg->Cycle.Inertial;
    leg->Cycle.CurrStepInd     = start_step % leg->Cycle.Inertial.NumOfSteps;
    leg->Cycle.StepStartTimeUS = esp_timer_get_time();
}

static void advance_step(leg_t * leg)
{
    const int N = leg->Cycle.Cycle.NumOfSteps;
    const int i = leg->Cycle.CurrStepInd;
    const int next = (i + 1) % N;

    step_t * prev = &leg->Cycle.Cycle.Steps[i];
    step_t * curr = &leg->Cycle.Cycle.Steps[next];

    leg->Cycle.CurrStepInd     = next;
    leg->Cycle.StepStartTimeUS = esp_timer_get_time();

    curr->start = prev->end;
    curr->end   = leg->Cycle.Inertial.Steps[next].end;
    update_step_coefficients(curr);
}

void gait_motion_tick(leg_t * leg)
{
    const int64_t now_us     = esp_timer_get_time();
    const int64_t elapsed_us = now_us - leg->Cycle.StepStartTimeUS;
    const int64_t step_us    = (int64_t)(STDSTEPTIME * 1e6f);

    if (elapsed_us >= step_us) {
        advance_step(leg);
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