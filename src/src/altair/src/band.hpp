
#ifndef __AKAME_BAND

#define __AKAME_BAND

#include "nearest.hpp"
#include "common.h"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "geometry_msgs/msg/point.hpp"

#include <math.h>
#include <stdio.h>

/* the first two here are checking for straighforward illigal
 * floating values. The last guy takes advantage of a property
 * that a nan or inf will never compare equal to itself.
 *
 * The x != x check I think is one of the only good cross
 * platform ways to determine floating point nan/inf, though
 * it is very odd to look at...                              */
#define BAD_VAL(x) (std::isnan(x) || std::isinf(x) || x != x)

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

      node * curr = path;

      double travel = 0.0;
      int count = 0;
      while (curr) {
         geometry_msgs::msg::PoseStamped pose;
         pose.pose.position.x = curr->pos.x;
         pose.pose.position.y = curr->pos.y;

         msg.poses.push_back(pose);

         travel += curr->n;
         count += 1;
         curr = curr->next;

         if (travel > extent && count >= 2)
            break;
      }
   }

   void to_vis(visualization_msgs::msg::MarkerArray & markers) {

      node * curr = path;

      int count = 0;
      while (curr) {
         visualization_msgs::msg::Marker marker;

         double rad = curr->d;
         if (rad == 0.0)
            rad = d0;

         marker.ns = "elastic_vis";
         marker.id = count;
         marker.type = visualization_msgs::msg::Marker::SPHERE;
         marker.action = 0; // add/modify
         marker.pose.position.x = curr->pos.x;
         marker.pose.position.y = curr->pos.y;
         marker.pose.position.z = 0;
         marker.scale.x = rad * 2.0;
         marker.scale.y = rad * 2.0;
         marker.scale.z = 0.1;
         marker.lifetime.sec = 1;
         marker.color.r = 0.0;
         marker.color.g = 1.0;
         marker.color.b = 1.0;
         marker.color.a = 0.5;

         markers.markers.push_back(marker);

         count += 1;
         curr = curr->next;
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
         next->d = bubble(next->pos);

         next->prev = path;
         path->next = next;
         path = next;
      }

      path = bootstrap.next;
      path->prev = NULL;
   }

#define ADVANCE(path)        \
do {                         \
                             \
   node * next = path->next; \
   free(path);               \
   path = next;              \
   if (path)                 \
      path->prev = NULL;     \
                             \
} while (0)

   void step(double px, double py, uint32_t count) {

      while (path && dist(px,py,path->pos.x,path->pos.y) < collide) {
         ADVANCE(path);
      }

      do {

         /* setup the system */
         node * q0, * q, * q1;
         q0 = path;
         if (q0 == NULL)
            return;
         q = path->next;
         if (q == NULL)
            return;
         q1 = q->next;
         if (q1 == NULL)
            return;

         double travel = 0.0;

         q0->d = bubble(q0->pos);
         while (q1) {
            point_val nearest = obs.get(q->pos);
            point fe = { 0, 0 };
            if (nearest.val < d0) {
               if (is_collision(nearest.val)) {
                  clear();
                  return;
               }
               fe = external(nearest.p,q->pos);
               q->d = nearest.val;
            }
            else q->d = d0;
            point fi = internal(q0->pos,q->pos,q1->pos);
            if (BAD_VAL(fi.x) || BAD_VAL(fi.y)) {
               clear(); return; /* this seems to occur on collision... */
            }
            point f = { fe.x + fi.x, fe.y + fi.y };
            double fc = constraint(q0->pos,q1->pos,f);

            q->pos.x += fc * f.x;
            q->pos.y += fc * f.y;

            double nextto = dist(q0->pos.x,q0->pos.y,q->pos.x,q->pos.y);
            q0->n = nextto;

            q0 = q; q = q1; q1 = q1->next;
            travel += nextto;
            if (travel > extent)
               break;
         }
         q->d = bubble(q->pos);

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
            else if (dist(q0->pos.x,q0->pos.y,q->pos.x,q->pos.y) > (q0->d + q->d)*0.9) {

               node * qnew = (node *)calloc(sizeof(node),1);
               qnew->pos.x = (q0->pos.x + q->pos.x) / 2.0;
               qnew->pos.y = (q0->pos.y + q->pos.y) / 2.0;
               qnew->d = bubble(qnew->pos);
               qnew->prev = q0;
               qnew->next = q;
               q0->next = qnew;
               q->prev = qnew;

               q0 = q; q = q1; q1 = q1->next;
            }
            else {
               q0 = q; q = q1; q1 = q1->next;
            }

            if (travel > extent)
               break;
            travel += q->n;
         }
         if (dist(q0->pos.x,q0->pos.y,q->pos.x,q->pos.y) > q0->d + q->d) {
            node * qnew = (node *)calloc(sizeof(node),1);
            qnew->pos.x = (q0->pos.x + q->pos.x) / 2.0;
            qnew->pos.y = (q0->pos.y + q->pos.y) / 2.0;
            qnew->d = bubble(qnew->pos);
            qnew->prev = q0;
            qnew->next = q;
            q0->next = qnew;
            q->prev = qnew;
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

      return point{ -d_div.x * d_d0 * kr, -d_div.y * d_d0 * kr};
   }

   /* motion constraint force. Normally doesn't have a multiplier but I am including it */
   inline double constraint(const point & qprev, const point & qnext, const point & f) const {
      double c = std::fabs( qprev.y - qnext.y );
      double b = std::fabs( qprev.x - qnext.x );
      double a = std::sqrt( c*c + b*b );
      double B = std::asin( b / a);
      double A = std::atan2(f.y,f.x);
      double x = std::fabs(A - B);

      /* this computes the force based on the distance from PI/2 */
      double fc = 1.0 - (km * ( std::fabs( std::fmod(x,M_PI) - (M_PI / 2.0)) / (M_PI / 2.0)));

      return std::fmax(fc,0.0);
   }

   inline double bubble(point q) {
      point_val pv = obs.get(q);
      if (pv.val > d0)
         return d0;
      return pv.val;
   }

   inline bool is_collision(point q) {
      point_val pv = obs.get(q);
      if (pv.val < collide)
         return true;
      return false;
   }

   inline bool is_collision(double d) {
      if (d < collide)
         return true;
      return false;
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
