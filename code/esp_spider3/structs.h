#ifndef STRUCTS_H
#define STRUCTS_H

#define STDNUMOFSTEPS 4
#define STDSTEPTIME   1.0f

typedef struct
{
  int   ServoIndexes[3];
  float LinkLengths[3];
  float JointOffsets[3];
  float BaseTranOffsets[3];
  float BaseRotOffsets[3];
  float JointHome[3];
  float EFHome[3];
} leg_t;

typedef struct
{
  leg_t Legs[6];
} spider_t;

typedef struct
{
  float pos[3];
  float vels[3];
} point_t;

typedef struct
{
  point_t start; // in angles
  point_t end;   // in angles
  float   coefficients[3][4]; // Hermite interpolation coefs
  float   StepTime;
} step_t;

typedef struct
{
  int     NumOfSteps;
  step_t  Steps[STDNUMOFSTEPS];
} cycle_t;

/* ---------------------------------------------------------------- */
/*  point_init                                                      */
/*                                                                  */
/*  Copies a pos[3] and vel[3] into a point_t.                      */
/*  If vel is NULL, velocities are set to zero.                     */
/* ---------------------------------------------------------------- */
void point_init(point_t * point, const float * pos, const float * vel);

/* ---------------------------------------------------------------- */
/*  cycle_init_step                                                 */
/*                                                                  */
/*  Writes a single step: start/end points and StepTime, then       */
/*  computes the cubic Hermite coefficients for that step.          */
/*                                                                  */
/*  Polynomial (local time t ∈ [0, StepTime]):                      */
/*                                                                  */
/*      angle(t) = a·t³ + b·t² + c·t + d                            */
/*                                                                  */
/*  Coefficients stored in step->coefficients[joint][k]:            */
/*      [j][0] = a   (t³ term)                                      */
/*      [j][1] = b   (t² term)                                      */
/*      [j][2] = c   (t¹ term)                                      */
/*      [j][3] = d   (constant)                                     */
/*                                                                  */
/*  Derivation (with t1 = 0, h = StepTime, α = joint angle,         */
/*  α' = joint angular velocity):                                   */
/*                                                                  */
/*      a = (α₁' + α₂') / h²  −  2·Δα / h³                          */
/*      b = 3·Δα / h²  −  (2·α₁' + α₂') / h                         */
/*      c = α₁'                                                     */
/*      d = α₁                                                      */
/* ---------------------------------------------------------------- */
void cycle_init_step (cycle_t * cycle, int step_index, const point_t * start, const point_t * end, const float step_time);

/* Compute the cubic Hermite coefficients for a step, in local time t ∈ [0, StepTime].
 *
 * The stored polynomial is:
 *     angle(t) = a·t³ + b·t² + c·t + d
 *
 * Layout in coefficients[joint][k]:
 *     k = 0 → a  (t³ coefficient)
 *     k = 1 → b  (t² coefficient)
 *     k = 2 → c  (t¹ coefficient)
 *     k = 3 → d  (constant term)
 *
 * This matches the ESP32 evaluation order:
 *     angle = ((a * t + b) * t + c) * t + d;
 *
 * Inputs are read from step->start and step->end (pos = angle in rad, vels = ang vel in rad/s).
 * Output is written to step->coefficients.
 */
void update_step_coefficients(step_t * step);

/* ---------------------------------------------------------------- */
/*  cycle_init                                                      */
/*                                                                  */
/*  Fill an entire cycle from a pre-computed array of trajectory    */
/*  points. Points are assumed to be in joint-space (angles +       */
/*  angular velocities) — run doIK/doJacobian before calling this.  */
/*                                                                  */
/*  Writes NumOfSteps + 1 points, generating NumOfSteps segments:   */
/*      step[i].start = points[i]                                   */
/*      step[i].end   = points[i + 1]                               */
/*      step[i].StepTime = step_time                                */
/*      step[i].coefficients = cubic Hermite from the above         */
/*                                                                  */
/*  `points` must have at least NumOfSteps + 1 valid entries.       */
/*  Caller must ensure cycle->Steps is sized for NumOfSteps.        */
/* ---------------------------------------------------------------- */
void cycle_init(cycle_t * cycle, int NumOfSteps, const point_t * points, const float step_time);

#endif //STRUCTS_H