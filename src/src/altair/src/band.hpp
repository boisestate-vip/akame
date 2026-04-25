
#ifndef __AKAME_BAND

#define __AKAME_BAND

#include "nearest.hpp"
#include "common.h"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <math.h>
#include <stdio.h>

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

      printf("publishing path\n");
      node * curr = path;

      double travel = 0.0;
      int count = 0;
      while (curr) {
         geometry_msgs::msg::PoseStamped pose;
         pose.pose.position.x = curr->pos.x;
         pose.pose.position.y = curr->pos.y;

         printf("adding node (%f, %f) to path\n",curr->pos.x,curr->pos.y);

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

      double travel = 0.0;
      int count = 0;
      while (curr) {
         visualization_msgs::msg::Marker marker;

         double rad = curr->d;
         if (rad == 0.0)
            rad = d0;

         marker.ns = "elastic_vis";
         marker.id = count;
         marker.type = visualization_msgs::msg::Marker::SPHERE;
         marker.action = 0; /* add/modify */
         marker.pose.position.x = curr->pos.x;
         marker.pose.position.y = curr->pos.y;
         marker.pose.position.z = 0;
         marker.scale.x = rad;
         marker.scale.y = rad;
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

      printf("loading path...\n");

      for (auto & pose : msg.poses) {
         node * next = (node *)calloc(sizeof(node),1);

         next->pos.x = pose.pose.position.x;
         next->pos.y = pose.pose.position.y;

         next->prev = path;
         path->next = next;
         path = next;
      }

      printf("finishing\n");

      path = bootstrap.next;
      path->prev = NULL;
      printf("finished\n");
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

      while (dist(px,py,path->pos.x,path->pos.y) < collide) {
         printf("advancing past %f %f\n",path->pos.x,path->pos.y);
         ADVANCE(path);
      }

      do {
         printf("stepping.\n");

         /* setup the system */
         node * q0, * q, * q1;
         q0 = path;
         q = path->next;
         printf("q: %p\n",q);
         if (q == NULL)
            return;
         q1 = q->next;
         printf("q1: %p\n",q1);
         if (q1 == NULL)
            return;

         double travel = 0.0;

         q0->d = bubble(q0->pos);
         while (q1) {
            point_val nearest = obs.get(q->pos);
            point fe = { 0, 0 };
            if (nearest.val < d0)
               fe = external(nearest.p,q->pos);
            point fi = internal(q0->pos,q->pos,q1->pos);
            point fc = constraint(q0->pos,q->pos,q1->pos);
            point f = { fe.x + fi.x + fc.x, fe.y + fi.y + fc.y };
            printf("applying force %f,%f to %f,%f\n",f.x,f.y,q->pos.x,q->pos.y);

            q->pos.x += f.x;
            q->pos.y += f.y;

            double nextto = dist(q0->pos.x,q0->pos.y,q->pos.x,q->pos.y);
            q0->n = nextto;

            q0 = q; q = q1; q1 = q1->next;
            travel += nextto;
            printf("adding %f to travel: %f\n",nextto,travel);
            if (travel > extent)
               break;
         }
         q->d = bubble(q->pos);

         q0 = path; q = q0->next; q1 = q->next;

         printf("culling/birthing\n");

         travel = 0.0;
         while (q1) {

            double between = dist(q0->pos.x,q0->pos.y,q1->pos.x,q1->pos.y);
            if ((q0->d + q1->d)*0.85 > between) {
               printf("culling %f,%f\n",q->pos.x,q->pos.y);

               free(q);

               q0->next = q1;
               q1->prev = q0;

               q = q1; q1 = q1->next;
            }
            else if (q->n > q0->d + q->d) {

               node * qnew = (node *)calloc(sizeof(node),1);
               qnew->pos.x = (q0->pos.x + q->pos.x) / 2.0;
               qnew->pos.y = (q0->pos.y + q->pos.y) / 2.0;
               qnew->d = bubble(qnew->pos);
               qnew->prev = q0;
               qnew->next = q;

               printf("birthing %f,%f\n",qnew->pos.x,qnew->pos.y);

               q0 = q; q = q1; q1 = q1->next;
            }
            else {
               q0 = q; q = q1; q1 = q1->next;
            }

            travel += q->n;
            printf("adding %f to travel: %f\n",q->n,travel);
            if (travel > extent)
               break;
         }
      } while (--count);
   }
#undef ADVANCE

   void clear() {
      printf("clearing previous path\n");
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

   inline double bubble(point q) {
      point_val pv = obs.get(q);
      if (pv.val > d0)
         return d0;
      return pv.val;
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
