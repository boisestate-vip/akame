
#ifndef __AKAME_BAND

#define __AKAME_BAND

#include "nearest.hpp"
#include "common.h"

#include "nav_msgs/msg/Path.hpp"

#include <math.h>

class Band {
public:

   Band(double internal_multiplier, double external_multiplier,
        double constraint_multiplier, double distance_cutoff) {
      this->ke = internal_multiplier;
      this->kr = external_multiplier;
      this->km = constraint_multiplier;
   }

   ~Band() {
   }

   void load(const nav_msgs::msg::Path & msg) {
      clear();

   }

   void step(uint32_t count) {
   }

   void clear() {
   }

private:

   /* elastic force */
   inline const point internal(const point & qprev, const point & q, const point & qnext) const {

      point diff_next = (point){.x = (qnext.x - q.x), .y = (qnext.y - q.y)};
      point diff_prev = (point){.x = (qprev.x - q.x), .y = (qprev.y - q.y)};
      double len_next = std::sqrt(diff_next.x*diff_next.x + diff_next.y*diff_next.y);
      double len_prev = std::sqrt(diff_prev.x*diff_prev.x + diff_prev.y*diff_prev.y);

      point force = (point){.x = (diff_next.x / len_next) + (diff_prev.x / len_prev),
                            .y = (diff_next.y / len_next) + (diff_prev.y / len_prev)};

      return (point){.x = force.x * ke, .y = force.y * ke};
   }

   /* repulsive force */
   inline const point external(const point & d, const point & q) const {
      point D = (point){.x = d.x-q.x, .y = d.y-q.y};
      double dist = sqrt( D.x*D.x + D.y*D.y );
      double d_d0 = d0 - dist;

      point d_div = (point){.x = D.x / dist, .y = D.y / dist};

      return (point){.x = d_div.x * d_d0 * kr, .y = d_div.y * d_d0 * kr};
   }

   /* motion constraint force. Normally doesn't have a multiplier but I am including it */
   inline const point constraint(const point & qprev, const point & q, const point & qnext) const {
      point diff_next = (point){.x = qnext.x - q.x, .y = qnext.y - q.y};
      point diff_prev = (point){.x = q.x - qprev.x, .y = q.y - qprev.y};
      double dist_next = sqrt( diff_next.x*diff_next.x + diff_next.y*diff_next.y );
      double dist_prev = sqrt( diff_prev.x*diff_prev.x + diff_prev.y*diff_prev.y );

      point force = (point){.x = (diff_next.x / dist_next) + (diff_prev.x / dist_prev),
                            .y = (diff_next.y / dist_next) + (diff_prev.y / dist_prev)};

      return (point){.x = force.x * km, .y = force.y * km};
   }

   /* distance cutoff for repulsive force */
   double d0;

   /* internal force multiplier */
   double ke;
   /* external force multiplier */
   double kr;
   /* motion constraint multiplier */
   double km;

};


#endif
