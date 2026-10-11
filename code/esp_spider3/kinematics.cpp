#include "kinematics.h"
#include <math.h>

/* Helper Functions */

/* ---------------------------------------------------------------- */
/*  3x3 linear solve via Cramer's rule                              */
/*                                                                  */
/*  Solves A * x = b for a 3x3 A. Returns false if |det| < eps.     */
/*  For our small fixed-size problem this beats Gauss elimination   */
/*  with partial pivoting: no branches, no pivoting table, and the  */
/*  DLS matrix is symmetric positive definite anyway, so it's never */
/*  actually close to singular.                                     */
/* ---------------------------------------------------------------- */
static bool solve3x3(const float A[3][3], const float b[3], float x[3])
{
    const float a = A[0][0], b0 = A[0][1], c  = A[0][2];
    const float d = A[1][0], e  = A[1][1], f  = A[1][2];
    const float g = A[2][0], h  = A[2][1], i  = A[2][2];

    const float ei_fh = e*i - f*h;
    const float di_fg = d*i - f*g;
    const float dh_eg = d*h - e*g;

    const float det = a*ei_fh - b0*di_fg + c*dh_eg;
    if (fabsf(det) < 1e-9f) return false;
    const float inv_det = 1.0f / det;

    x[0] = ( b[0]*ei_fh
           - b0   *(b[1]*i - f*b[2])
           + c    *(b[1]*h - e*b[2])) * inv_det;

    x[1] = ( a    *(b[1]*i - f*b[2])
           - b[0] *(di_fg)
           + c    *(d*b[2] - b[1]*g)) * inv_det;

    x[2] = ( a    *(e*b[2] - b[1]*h)
           - b0   *(d*b[2] - b[1]*g)
           + b[0] *(dh_eg)          ) * inv_det;

    return true;
}

/* ---------------------------------------------------------------- */
/*  Jacobian at one joint configuration                             */
/*                                                                  */
/*  J maps joint velocities (rad/s) to Cartesian velocity of the    */
/*  foot. Evaluated at a joint configuration in radians.            */
/* ---------------------------------------------------------------- */
static void computeJacobian(const leg_t * leg, const float q[3], float J[3][3])
{
    const float L1 = leg->LinkLengths[0];
    const float L2 = leg->LinkLengths[1];
    const float L3 = leg->LinkLengths[2];

    const float phi1 = q[0] + leg->JointOffsets[0];
    const float phi2 = q[1] + leg->JointOffsets[1];
    const float phi3 = q[2] + leg->JointOffsets[2];

    const float S1 = sinf(phi1), C1 = cosf(phi1);
    const float S2 = sinf(phi2), C2 = cosf(phi2);

    const float phi23 = phi3 - phi2;
    const float S23   = sinf(phi23), C23 = cosf(phi23);

    const float R = L1 + L2*S2 + L3*S23;
    const float Q = L2*C2 - L3*C23;
    const float P = L3*C23;

    J[0][0] =  R * S1;
    J[0][1] = -Q * C1;
    J[0][2] = -P * C1;

    J[1][0] =  R * C1;
    J[1][1] =  Q * S1;
    J[1][2] =  P * S1;

    J[2][0] =  0.0f;
    J[2][1] = -(L2*S2 + L3*S23);
    J[2][2] =   L3*S23;
}

/* ---------------------------------------------------------------- */
/*  Damped least squares:  dq = J^T (J J^T + λ²I)^-1 v              */
/* ---------------------------------------------------------------- */
static bool solveJacobianDLS(const float J[3][3], const float v[3], float dq[3])
{
    float norm2 = 0.0f;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            norm2 += J[i][j] * J[i][j];

    if  (norm2 < 0) {return false;}

    /* λ² = k * ||J||², k ~ 1e-2 */
    const float lambda_sq = 1e-2f * norm2;

    /* A = J J^T + λ²I  (symmetric) */
    float A[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) s += J[i][k] * J[j][k];
            A[i][j] = s;
        }
        A[i][i] += lambda_sq;
    }

    /* Solve A * y = v */
    float y[3];
    if (!solve3x3(A, v, y)) {return false;}

    /* dq = J^T * y */
    for (int j = 0; j < 3; j++) {
        dq[j] = J[0][j]*y[0] + J[1][j]*y[1] + J[2][j]*y[2];
    }
    return true;
}

/* ---------------------------------------------------------------- */
/*  Public API                                                      */
/* ---------------------------------------------------------------- */

ik_status_t doIK(const leg_t * Leg, int num_of_pts, point_t * points)
{
    const float L1 = Leg->LinkLengths[0], L2 = Leg->LinkLengths[1], L3 = Leg->LinkLengths[2];
    const float L1_sq = L1 * L1;
    const float L2_sq = L2 * L2;
    const float L3_sq = L3 * L3;

    /* Reciprocal constants: hoisted so the inner loop multiplies
     * instead of dividing. Division on ESP32 FPU is ~14 cycles. */
    const float inv_2L2    = 1.0f / (2.0f * L2);
    const float inv_2L2L3  = 1.0f / (2.0f * L2 * L3);

    const float J1_off = Leg->JointOffsets[0];
    const float J2_off = Leg->JointOffsets[1];
    const float J3_off = Leg->JointOffsets[2];

    ik_status_t status = IK_OK;

    for (int i = 0; i < num_of_pts; i++) {
        const float X = points[i].pos[0];
        const float Y = points[i].pos[1];
        const float Z = points[i].pos[2];

        const float L23_xy = sqrtf(X*X + Y*Y) - L1;

        if (L23_xy < 0) {status = IK_OUT_OF_REACH; continue;}

        const float L23_sq = L23_xy * L23_xy + Z * Z;
        const float L23    = sqrtf(L23_sq);

        /* Degenerate case: target on the L1 axis. atan2(0,0) is
         * undefined and division by L23 blows up. */
        if (L23 < 1e-5f) {status = IK_DEGENERATE; continue;}

        const float inv_L23 = 1.0f / L23;

        /* theta = interior angle at the knee, via law of cosines */
        float cos_theta = (L2_sq + L23_sq - L3_sq) * inv_2L2 * inv_L23;
        if (cos_theta >  1.0f) { cos_theta =  1.0f; status = IK_OUT_OF_REACH; }
        if (cos_theta < -1.0f) { cos_theta = -1.0f; status = IK_OUT_OF_REACH; }
        const float theta = acosf(cos_theta);

        /* sum = interior angle of the two-link chain */
        float cos_sum = (L2_sq + L3_sq - L23_sq) * inv_2L2L3;
        if (cos_sum >  1.0f) { cos_sum =  1.0f; status = IK_OUT_OF_REACH; }
        if (cos_sum < -1.0f) { cos_sum = -1.0f; status = IK_OUT_OF_REACH; }
        const float sum = acosf(cos_sum);

        points[i].pos[0] = atan2f(Y, -X)             - J1_off;
        points[i].pos[1] = atan2f(L23_xy, Z) - theta - J2_off;
        points[i].pos[2] = sum                       - J3_off;
    }

    return status;
}

ik_status_t doJacobian(const leg_t * leg, int num_of_pts, point_t * points)
{
    ik_status_t status = IK_OK;

    for (int i = 0; i < num_of_pts; i++) {
        /* points[i].pos currently holds joint ANGLES (rad) — result of doIK */
        float J[3][3];
        computeJacobian(leg, points[i].pos, J);

        /* points[i].vel currently holds CARTESIAN velocity */
        float dq[3];
        if (!solveJacobianDLS(J, points[i].vels, dq)) {
            points[i].vels[0] = 0.0f;
            points[i].vels[1] = 0.0f;
            points[i].vels[2] = 0.0f;
            status = IK_DEGENERATE;
            continue;
        }

        /* Replace with joint velocity in rad/s */
        points[i].vels[0] = dq[0];
        points[i].vels[1] = dq[1];
        points[i].vels[2] = dq[2];
    }

    return status;
}