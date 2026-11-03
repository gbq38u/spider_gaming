#include "awakeness.h"
#include "esp_node.h"
#include "kinematics.h"

#include <math.h>
#include <esp_timer.h>
#include <Arduino.h>

#define WAKE_STAND_DURATION  3.0f
#define WAKE_SERVO_TICK_MS   20

static bool build_wake_step(leg_t * leg, step_t * out_step)
{
  point_t home_q, stand_q;

  for (int j = 0; j < 3; j++) {
      home_q.pos[j]  = leg->Geometry.JointHome[j];
      home_q.vels[j] = 0.0f;
  }
  for (int j = 0; j < 3; j++) {
      stand_q.pos[j]  = leg->Geometry.EFHome[j];
      stand_q.vels[j] = 0.0f;
  }

  ik_status_t st = doIK(leg, 1, &stand_q);
  if (st == IK_DEGENERATE) return false;

  step_init(out_step, &home_q, &stand_q, WAKE_STAND_DURATION);
  return true;
}

static void play_steps_blocking(leg_t * const * legs, const step_t * steps, int count, float duration, servo_write_fn_t write_fn)
{
  const int64_t t0     = esp_timer_get_time();
  const int64_t dur_us = (int64_t)(duration * 1e6f);

  while (esp_timer_get_time() - t0 < dur_us) {
    float t = (float)(esp_timer_get_time() - t0) / 1e6f;
    if (t > duration) t = duration;

    for (int i = 0; i < count; i++) {
      const step_t * s = &steps[i];
      leg_t * leg = legs[i];

      for (int j = 0; j < 3; j++) {
          const float q = ((s->coefficients[j][0] * t + s->coefficients[j][1]) * t + s->coefficients[j][2]) * t + s->coefficients[j][3];
          leg->Servos.ServoAngles[j] = constrainf(q * RAD2DEG, 0.0f, 180.0f);
      }
      write_fn(leg);
    }
    delay(WAKE_SERVO_TICK_MS);
  }

  /* Snap to exact endpoint. */
  for (int i = 0; i < count; i++) {
    const step_t * s = &steps[i];
    leg_t * leg = legs[i];
    for (int j = 0; j < 3; j++) {
      leg->Servos.ServoAngles[j] = constrainf(s->end.pos[j] * RAD2DEG, 0.0f, 180.0f);
    }
    write_fn(leg);
  }
}

static bool stand_group(spider_t * spider, const int * indices, int count, servo_write_fn_t write_fn)
{
    leg_t * legs[3];
    step_t  steps[3];

    for (int i = 0; i < count; i++) {
        legs[i] = spider->Legs[indices[i]];
        if (!build_wake_step(legs[i], &steps[i])) return false;
    }
    play_steps_blocking(legs, steps, count, WAKE_STAND_DURATION, write_fn);
    return true;
}

bool spider_wake_up_sequence(spider_t * spider, servo_write_fn_t write_fn)
{
    /* Left tripod: LF=0, LM=2, LR=4 */
    static const int left_idx[3]  = { 0, 2, 4 };
    /* Right tripod: RF=1, RM=3, RR=5 */
    static const int right_idx[3] = { 1, 3, 5 };

    if (!stand_group(spider, left_idx,  3, write_fn)) return false;
    if (!stand_group(spider, right_idx, 3, write_fn)) return false;
    return true;
}