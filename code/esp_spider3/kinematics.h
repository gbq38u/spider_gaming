#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "structs.h"

typedef enum {
    IK_OK = 0,
    IK_OUT_OF_REACH,
    IK_DEGENERATE,
} ik_status_t;

// changes points so yesterday's pos  are today's angs
ik_status_t doIK      (const leg_t * Leg, int num_of_pts, point_t * points);
// changes points so yesterday's vels are today's ang_vels
ik_status_t doJacobian(const leg_t * Leg, int num_of_pts, point_t * points);



#endif //KINEMATICS_H