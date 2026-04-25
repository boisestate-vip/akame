
#ifndef __AKAME_NEAREST

#define __AKAME_NEAREST

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "common.h"
#include <float.h>
#include <stdint.h>

#include <vector>
#include <cmath>

/* start asap, more complex later... */

class Nearest {
public:

   Nearest() {
   }

   Nearest(uint8_t map_height, double bin_width) {
      this->map_height = map_height;
      this->bin_width = bin_width;
   }

   ~Nearest() {
   }

   void load(const nav_msgs::msg::OccupancyGrid & map) {
      clear();

      double step = map.info.resolution;
      uint32_t xmax = map.info.width;
      uint32_t ymax = map.info.height;

      uint8_t * idx = (uint8_t *)map.data.data();

      double x = map.info.origin.position.x;
      double y = map.info.origin.position.y;
      for (uint32_t xstp = 0; xstp < xmax; ++xstp) {
         for (uint32_t ystp = 0; ystp < ymax; ++ystp) {

            if (*idx > map_height)
               values.push_back(point{x,y});

            idx += 1;

            x += step;
         }
         y += step;
      }
   }

   point_val get(point p) {
      point best = p;
      double best_dist2 = DBL_MAX, curr_dist2;

      point * val = (point *)values.data(), * end = (point *)values.data() + values.size();
      while (val < end) {
         if ((curr_dist2 = dist2(p,*val)) < best_dist2) {
            best_dist2 = curr_dist2;
            best = *val;
         }

         val += 1;
      }

      return point_val{best,std::sqrt(best_dist2)};
   }

   void clear() {
      values.clear();
   }

private:

   uint8_t map_height;
   double bin_width;

   std::vector<point> values;

   inline double dist2(const point & a, const point & b) const {
      return (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y);
   }

};

#endif
