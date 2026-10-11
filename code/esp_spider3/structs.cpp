#include "structs.h"

#include <string.h>   /* for memcpy, if you prefer it over element-wise copies */


void update_step_coefficients(step_t * step)
{
    if (step == NULL) { return; }

    const float h = step->StepTime;
    if (h <= 0.0f) { return; }

    const float inv_h  = 1.0f / h;
    const float inv_h2 = inv_h  * inv_h;
    const float inv_h3 = inv_h2 * inv_h;

    for (int j = 0; j < 3; j++) {
        const float a1  = step->start.pos[j];    /* α₁  */
        const float a2  = step->end.pos[j];      /* α₂  */
        const float a1d = step->start.vels[j];   /* α₁' */
        const float a2d = step->end.vels[j];     /* α₂' */

        const float delta = a2 - a1;

        const float a = (a1d + a2d) * inv_h2 - 2.0f * delta * inv_h3;
        const float b = 3.0f * delta * inv_h2 - (2.0f * a1d + a2d) * inv_h;
        const float c = a1d;
        const float d = a1;

        step->coefficients[j][0] = a;
        step->coefficients[j][1] = b;
        step->coefficients[j][2] = c;
        step->coefficients[j][3] = d;
    }
}

void point_init(point_t * point, const float * pos, const float * vel)
{
    if (point == NULL || pos == NULL) { return; }

    point->pos[0] = pos[0];
    point->pos[1] = pos[1];
    point->pos[2] = pos[2];

    if (vel != NULL) {
        point->vels[0] = vel[0];
        point->vels[1] = vel[1];
        point->vels[2] = vel[2];
    } else {
        point->vels[0] = 0.0f;
        point->vels[1] = 0.0f;
        point->vels[2] = 0.0f;
    }
}

void cycle_init_step(cycle_t * cycle, int step_index, const point_t * start, const point_t * end, const float step_time)
{
    if (cycle == NULL || start == NULL || end == NULL) { return; }
    if (step_index < 0 || step_index >= cycle->NumOfSteps) { return; }
    if (step_time <= 0.0f) { return; }

    step_t * step = &cycle->Steps[step_index];

    /* Store endpoints and timing */
    step->start    = *start;
    step->end      = *end;
    step->StepTime = step_time;

    /* Compute coefficients from the freshly-stored endpoints */
    update_step_coefficients(step);
}

void cycle_init(cycle_t * cycle, int NumOfSteps, const point_t * points, const float step_time)
{
    if (cycle == NULL || points == NULL)    { return; }
    if (NumOfSteps < 1)                     { return; }
    if (NumOfSteps > STDNUMOFSTEPS)         { return; }
    if (step_time <= 0.0f)                  { return; }

    cycle->NumOfSteps = NumOfSteps;

    for (int i = 0; i < NumOfSteps; i++) {
        cycle_init_step(cycle, i, &points[i], &points[i + 1], step_time);
    }
}