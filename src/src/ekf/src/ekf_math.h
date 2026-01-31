

#ifndef __EKF_MATH

#define __EKF_MATH
#include <math.h>

#define NORMALIZE_ANGLE(theta) atan2(sin((theta)),cos((theta)))

typedef struct ekf_pos {
   double x;
   double y;
   double o; // theta
} ekf_pos;

typedef struct ekf_vel {
   double v;
   double w;
} ekf_vel; 

typedef struct ekf_jacobion {
   double xwt;
   double ywt;
} ekf_jacobian;

ekf_pos estimate_new_pos(ekf_pos old_pos, ekf_vel curr_vel, double delta);
ekf_jacobian estimate_new_jacobian(double o, ekf_vel curr_vel, double delta);

#endif // __EKF_MATH
