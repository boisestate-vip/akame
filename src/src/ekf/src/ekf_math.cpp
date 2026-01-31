
#include "ekf_math.h"

ekf_pos estimate_new_pos(ekf_pos old_pos,
                         ekf_vel curr_vel,
                         double delta) {

   double vw = curr_vel.v / curr_vel.w;
   double wt = curr_vel.w * delta;

   double sint = NORMALIZE_ANGLE(sin(old_pos.o));
   double cost = NORMALIZE_ANGLE(cos(old_pos.o));

   double sin_wt = NORMALIZE_ANGLE(sin(old_pos.o + wt));
   double cos_wt = NORMALIZE_ANGLE(cos(old_pos.o + wt));

   return (ekf_pos){
      .x = -(vw * sint) + (vw * sin_wt),
      .y =  (vw * cost) - (vw * cos_wt),
      .o = NORMALIZE_ANGLE(wt),
   };
}

ekf_jacobian estimate_new_jacobian(double o,
                                   ekf_vel curr_vel,
                                   double delta) {

   double vw = curr_vel.v / curr_vel.w;
   double wt = curr_vel.w * delta;

   double sint = NORMALIZE_ANGLE(sin(o));
   double cost = NORMALIZE_ANGLE(cos(o));

   double sin_wt = NORMALIZE_ANGLE(sin(o + wt));
   double cos_wt = NORMALIZE_ANGLE(cos(o + wt));

   return (ekf_jacobian){
      .xwt = (vw * cost) - (vw * cos_wt),
      .ywt = (vw * sint) - (vw * sin_wt),
   };
}
