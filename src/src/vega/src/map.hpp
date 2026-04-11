
#ifndef __AKAME_MAP_VEGA

#define __AKAME_MAP_VEGA
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <chrono>
#include <algorithm>

#define ROS2_MAPLOW    0
#define ROS2_MAPHIGH 100

class GridMap {
private:

   /* resolution of the map in meters */
   double resolution;

   /* position of the bottom corner of the map.
    * Multiply by the resolution to get this in
    * the 'real' space.                        */
   int32_t xmin;
   int32_t ymin;

   /* radius in indicies of the circle */
   double circle_rad_raw;
   uint32_t circle_rad;

   /* map data. Held as an array but represents a matrix */
   uint8_t * map = NULL;

   /* how much to increase measure on a obstacle hit */
   int32_t hit_weight;
   /* how much to decrease measure on a obstacle miss */
   int32_t miss_weight;
   /* value to place in the map at startup, 0 */
   int32_t start_weight;

public:

   /* the size of the map in the x and y directions */
   uint32_t xlen;
   uint32_t ylen;
   uint32_t len;


   GridMap() {} // dummy for resting initialization

   GridMap(double resolution, int32_t hit_weight, int32_t miss_weight, int32_t start_weight,
           double initial_xsize, double initial_ysize, double circle_rad) 
      : resolution(resolution), hit_weight(hit_weight), miss_weight(miss_weight),
        start_weight(start_weight) {


      int32_t start_xlen = std::ceil( initial_xsize / resolution );
      int32_t start_ylen = std::ceil( initial_ysize / resolution );

      this->xmin = -start_xlen / 2;
      this->ymin = -start_ylen / 2;
      this->xlen = start_xlen;
      this->ylen = start_ylen;

      this->map = (uint8_t *)malloc(sizeof(uint8_t)*this->xlen*this->ylen);
      memset(this->map,start_weight,sizeof(uint8_t)*this->xlen*this->ylen);
      this->len = this->xlen * this->ylen;

      this->circle_rad_raw = circle_rad;
      this->circle_rad = std::ceil( circle_rad / resolution );
   }

   double res() {
      return resolution;
   }

   void lens(uint32_t & x, uint32_t & y) {
      x = xlen;
      y = ylen;
   }

   int is_obs(int idx) {

      if (len <= idx)
         return 0;

      if (idx < 0)
         return 0;

      if (map[idx] <= hit_weight)
         return 1;

      return 0;
   }

   uint32_t getidx(double x, double y) {

      int32_t xi = (int32_t)(x / resolution) - xmin;
      int32_t yi = (int32_t)(y / resolution) - ymin;

      return xi + (yi * xlen);
   }

   void getidx2(double x, double y, int32_t & xi, int32_t & yi) {

      xi = (int32_t)(x / resolution) - xmin;
      yi = (int32_t)(y / resolution) - ymin;

   }

   void getpos(uint32_t idx, double & x, double & y) {

      int32_t xi = idx % xlen;
      int32_t yi = idx / xlen;

      x = (double)(xi + xmin) * resolution;
      y = (double)(yi + ymin) * resolution;
   }

   void getpos2(uint32_t xi, uint32_t yi, double & x, double & y) {
      x = (double)(xi + xmin) * resolution;
      y = (double)(yi + ymin) * resolution;
   }

   /* get the map value at the given position. No
    * out-of-bounds checking...                  */
   uint8_t & operator()(double x, double y) {

      int32_t xi = (int32_t)(x / resolution) - xmin;
      int32_t yi = (int32_t)(y / resolution) - ymin;

      return map[xi + (xlen*yi)];
   }

   /* grow to encompass the given size. Will only grow, never shrink 
    *
    * return true if growth happened                                */
   int grow_to(double xlow, double ylow, double xhigh, double yhigh) {

      /* compute these values relative to the current size of
       * the map. This has the effect of signaling pretty clearly
       * when we are out of bounds.                               */

      int32_t xlow_i  = (int32_t)(xlow / resolution)  - xmin;
      int32_t ylow_i  = (int32_t)(ylow / resolution)  - ymin;
      int32_t xhigh_i = (int32_t)(xhigh / resolution) - (xmin + xlen) + 1;
      int32_t yhigh_i = (int32_t)(yhigh / resolution) - (ymin + ylen) + 1;

      /* based on the above work, if any of the below is true
       * then we need to expand the map.                     */
      if (xlow_i < 0 ||
          ylow_i < 0 ||
          xhigh_i > 0 ||
          yhigh_i > 0) {

         /* compute our new sizes */
         int32_t new_xmin = xmin, new_xlen = xlen;
         int32_t new_ymin = ymin, new_ylen = ylen;

         if (xlow_i < 0) {
            new_xmin += xlow_i;
            new_xlen -= xlow_i;
         }

         if (ylow_i < 0) {
            new_ymin += ylow_i;
            new_ylen -= ylow_i;
         }

         if (xhigh_i > 0)
            new_xlen += xhigh_i;

         if (yhigh_i > 0)
            new_ylen += yhigh_i;


         /* allocate our new map */
         int32_t new_map_size = sizeof(uint8_t)*new_xlen*new_ylen;
         uint8_t * new_map = (uint8_t *)malloc(new_map_size);
         memset(new_map,start_weight,new_map_size);
         len = new_map_size;

         /* do the risky work of copying things over... */
         /* we put y in the outer loop because it allows us to keep cachelines in
          * memory longer.                                                       */
         for (uint32_t oldy = 0, newy = (ymin-new_ymin)*new_xlen; oldy < xlen*ylen; oldy += xlen, newy += new_xlen) {
            for (uint32_t oldx = 0, newx = xmin-new_xmin; oldx < xlen; ++oldx, ++newx) {
               new_map[newx + newy] = map[oldx + oldy];
            }
         }

         /* now swap over to the new values */
         xlen = new_xlen;
         ylen = new_ylen;
         xmin = new_xmin;
         ymin = new_ymin;

         free(map);
         map = new_map;

         return 1;
      }
      return 0;
   }

   /* add an observation (lidar or point cloud) starting at x/y s and ending
    * at x/y e. No checking for out of bounds in this function.             */
   void add_observation(double xs, double ys, double xe, double ye, int obstacle=1) {

      // what follows is an implimentation of
      // https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm#
      /* I will confess that I don't know a ton of the details about
       * this algorithm, only that it works well for these purposes */
      
      int32_t x0 = (int32_t)(xs / resolution) - xmin;
      int32_t y0 = (int32_t)(ys / resolution) - ymin;
      int32_t x1 = (int32_t)(xe / resolution) - xmin;
      int32_t y1 = (int32_t)(ye / resolution) - ymin;

      int32_t dx = std::abs(x1 - x0);
      int32_t sx = x0 < x1 ? 1 : -1;
      int32_t dy = -std::abs(y1 - y0);
      int32_t sy = y0 < y1 ? xlen : -xlen;
      int32_t error = dx + dy;

      y0 *= xlen;
      y1 *= xlen;

      while (1) {

         map[x0 + y0] = std::max(map[x0 + y0] - miss_weight, ROS2_MAPLOW);

         int32_t e2 = 2 * error;
         if (e2 >= dy) {
            if (x0 == x1) break;
            error = error + dy;
            x0 = x0 + sx;
         }
         if (e2 <= dx) {
            if (y0 == y1) break;
            error = error + dx;
            y0 = y0 + sy;
         }
      }

      /* extra step, add the hit entry 
       * We include the miss weight because
       * we need to undo our action of applying
       * it earlier (in the last iteration...) */
      if (obstacle)
         map[x0 + y0] = std::min(map[x0 + y0] + miss_weight + hit_weight, ROS2_MAPHIGH);
   }

   void clear() {
      memset(map,start_weight,xlen*ylen);
   }

   /* convert the occupancy grid to a ros2 message, filling
    * the provided message variable's relavant fields.      */
   void to_msg(nav_msgs::msg::OccupancyGrid & msg) {

      msg.info.resolution = resolution;
      msg.info.width = xlen;
      msg.info.height = ylen;

      msg.info.origin.position.x = (double)xmin * resolution;
      msg.info.origin.position.y = (double)ymin * resolution;
      msg.info.origin.position.z = 0.0;
      
      msg.info.origin.orientation.x = 0.0;
      msg.info.origin.orientation.y = 0.0;
      msg.info.origin.orientation.z = 0.0;
      msg.info.origin.orientation.w = 1.0;

      for (uint32_t i = 0; i < xlen*ylen; ++i)
         msg.data.push_back(map[i]);
   }

   /* replace the current map with the contents of a new
    * map using the given occupancy grid message.       
    *
    * returns true if the map grew/changed size.       */
   int from_msg(nav_msgs::msg::OccupancyGrid & msg) {

      resolution = msg.info.resolution;
      xlen = msg.info.width;
      ylen = msg.info.height;

      xmin = std::round(msg.info.origin.position.x / resolution);
      ymin = std::round(msg.info.origin.position.y / resolution);


      double extend = circle_rad * resolution;
      double xlow = msg.info.origin.position.x  + extend;
      double ylow = msg.info.origin.position.y  + extend;
      double xhigh = xlow + (xlen * resolution) + extend;
      double yhigh = ylow + (ylen * resolution) + extend;

      int res = grow_to(xlow,ylow,xhigh,yhigh);

      uint8_t * back = msg.data.data() + msg.data.size();
      uint8_t * pos = msg.data.data();
      uint32_t idx = 0;
      while (pos < back) {
         if (hit_weight <= *pos)
            internal_add_circle(idx,*pos);

         idx += 1;
         pos += 1;
      }

      return res;
   }

private:

   // https://en.wikipedia.org/wiki/Midpoint_circle_algorithm#Jesko's_method
   void internal_add_circle(uint32_t idx, uint8_t val) {

      uint32_t t1 = circle_rad / 16, t2;
      uint32_t x = circle_rad;
      uint32_t y = 0;
      uint32_t yidx = 0;
      uint32_t xidx = x * xlen;

      while (x >= y) {

         map[idx + x + yidx] = 100;
         map[idx - x + yidx] = 100;
         map[idx - x - yidx] = 100;
         map[idx + x - yidx] = 100;

         map[idx + y + xidx] = 100;
         map[idx - y + xidx] = 100;
         map[idx - y - xidx] = 100;
         map[idx + y - xidx] = 100;

         yidx += xlen;
         y += 1;

         t1 = t1 + y;
         t2 = t1 - x;
         if (t2 >= 0) {
            t1 = t2;
            x -= 1;
            xidx -= xlen;
         }

      }
   }

};

#endif
