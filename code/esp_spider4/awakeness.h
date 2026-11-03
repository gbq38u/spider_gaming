#ifndef WAKE_UP_H
#define WAKE_UP_H

#include "structs.h"

/* The caller supplies a function that takes a leg and writes
 * leg->Servos.ServoAngles to the hardware. This keeps wake_up.cpp
 * independent of any specific servo driver. */
typedef void (*servo_write_fn_t)(leg_t * leg);

/* Blocking wake-up sequence. Lifts the left tripod, then the right.
 * Returns true if all legs stood up successfully. */
bool spider_wake_up_sequence(spider_t * spider, servo_write_fn_t write_fn);

#endif // WAKE_UP_H