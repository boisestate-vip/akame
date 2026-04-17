
#ifndef __AKAME_BAND

#define __AKAME_BAND

#include "nearest.hpp"
#include "common.h"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <math.h>

static inline double dist(const double x0, const double y0, const double x1, const double y1) {
   return std::sqrt( (x0-x1)*(x0-x1) + (y0-y1)*(y0-y1) );
}

class Band {
public:

   Band() {
   }

   Band(double internal_multiplier, double external_multiplier,
        double constraint_multiplier, double distance_cutoff,
        uint8_t map_height, double bin_width, 
        double max_length, double hit_distance)
   : obs(Nearest(map_height,bin_width)) {
      this->ke = internal_multiplier;
      this->kr = external_multiplier;
      this->km = constraint_multiplier;
      this->d0 = distance_cutoff;
      this->extent = max_length;
      this->collide = hit_distance;
      this->path = NULL;
   }

   ~Band() {
      clear();
   }

   int has_path(void) {
      if (path == NULL)
         return 0;
      return 1;
   }

   void to_msg(nav_msgs::msg::Path & msg) {

      double travel = 0.0;
      while (path) {
         geometry_msgs::msg::PoseStamped pose;
         pose.pose.position.x = path->pos.x;
         pose.pose.position.y = path->pos.y;

         msg.poses.push_back(pose);

         path = path->next;
         if (travel > extent)
            break;
      }
   }

   void load_map(const nav_msgs::msg::OccupancyGrid & msg) {
      obs.load(msg);
   }

   void load_path(const nav_msgs::msg::Path & msg) {
      clear();
      static node bootstrap;
      path = &bootstrap;

      for (auto & pose : msg.poses) {
         node * next = (node *)calloc(sizeof(node),1);
   next->pos.x = pose.pose.position.x;
         next->pos.y = pose.pose.position.y;
         next->prev = path;
         path->next = next;
         path = next;
      }

      path = path->next;
      path->prev = NULL;
   }

#define ADVANCE(path)        \
do {                         \
                             \
   node * next = path->next; \
   free(path);               \
   path = next;              \
   path->prev = NULL;        \
                             \
} while (0)

   void step(double px, double py, uint32_t count) {

      while (dist(px,py,path->pos.x,path->pos.y) > collide) {
         ADVANCE(path);
      }

      do {
         /* setup the system */
         node * q0, * q, * q1;
         q0 = path;
         q = path->next;
         if (q == NULL)
            return;
         q1 = q->next;
         if (q1 == NULL)
            return;

         double travel = 0.0;

         q0->d = obs.get(q0->pos).val;
         while (q1) {
            point_val nearest = obs.get(q->pos);
            point fe = { 0, 0 };
            if (nearest.val < d0)
               fe = external(nearest.p,q->pos);
            point fi = internal(q0->pos,q->pos,q1->pos);
            point fc = constraint(q0->pos,q->pos,q1->pos);
            point f = { fe.x + fi.x + fc.x, fe.y + fi.y + fc.y };

            q->pos.x += f.x;
            q->pos.y += f.y;

            double nextto = dist(q0->pos.x,q0->pos.y,q->pos.x,q->pos.y);
            q0->n = nextto;

            q0 = q; q = q1; q1 = q1->next;
            travel += nextto;
            if (travel > extent)
               break;
         }
         q->d = obs.get(q->pos).val;

         q0 = path; q = q0->next; q1 = q->next;

         travel = 0.0;
         while (q1) {

            double between = dist(q0->pos.x,q0->pos.y,q1->pos.x,q1->pos.y);
            if ((q0->d + q1->d)*0.85 > between) {

               free(q);

               q0->next = q1;
               q1->prev = q0;

               q = q1; q1 = q1->next;
            }
            else if (q->n > q0->d + q->d) {

               node * qnew = (node *)calloc(sizeof(node),1);
               qnew->pos.x = (q0->pos.x + q->pos.x) / 2.0;
               qnew->pos.y = (q0->pos.y + q->pos.y) / 2.0;
               qnew->d = obs.get(qnew->pos).val;
               qnew->prev = q0;
               qnew->next = q;

               q0 = q; q = q1; q1 = q1->next;
            }
            else {
               q0 = q; q = q1; q1 = q1->next;
            }

            travel += q->n;
            if (travel > extent)
               break;
         }
      } while (--count);
   }
#undef ADVANCE

   void clear() {
      obs.clear();

      node * next;
      while (path) {
         next = path->next;

         free(path);
         path = next;
      }
   }

private:

   /* elastic force */
   inline const point internal(const point & qprev, const point & q, const point & qnext) const {

      point diff_next = point{ (qnext.x - q.x), (qnext.y - q.y)};
      point diff_prev = point{ (qprev.x - q.x), (qprev.y - q.y)};
      double len_next = std::sqrt(diff_next.x*diff_next.x + diff_next.y*diff_next.y);
      double len_prev = std::sqrt(diff_prev.x*diff_prev.x + diff_prev.y*diff_prev.y);

      point force = point{ (diff_next.x / len_next) + (diff_prev.x / len_prev),
                           (diff_next.y / len_next) + (diff_prev.y / len_prev)};

      return point{ force.x * ke, force.y * ke};
   }

   /* repulsive force */
   inline const point external(const point & d, const point & q) const {
      point D = point{ d.x-q.x, d.y-q.y};
      double dist = sqrt( D.x*D.x + D.y*D.y );
      double d_d0 = d0 - dist;

      point d_div = point{ D.x / dist, D.y / dist};

      return point{ d_div.x * d_d0 * kr, d_div.y * d_d0 * kr};
   }

   /* motion constraint force. Normally doesn't have a multiplier but I am including it */
   inline const point constraint(const point & qprev, const point & q, const point & qnext) const {
      point diff_next = point{ qnext.x - q.x, qnext.y - q.y};
      point diff_prev = point{ q.x - qprev.x, q.y - qprev.y};
      double dist_next = sqrt( diff_next.x*diff_next.x + diff_next.y*diff_next.y );
      double dist_prev = sqrt( diff_prev.x*diff_prev.x + diff_prev.y*diff_prev.y );

      point force = point{ (diff_next.x / dist_next) + (diff_prev.x / dist_prev),
                           (diff_next.y / dist_next) + (diff_prev.y / dist_prev)};

      return point{ force.x * km, force.y * km};
   }

   /* distance cutoff for repulsive force */
   double d0;

   /* total length the path is allowed to extend */
   double extent;
   /* the distance to mark a node as a hit at */
   double collide;

   /* internal force multiplier */
   double ke;
   /* external force multiplier */
   double kr;
   /* motion constraint multiplier */
   double km;

   /* used to find nearest obstacles */
   Nearest obs;

   /* the path being worked on. Will be null if not */
   node * path;
};


#endif
