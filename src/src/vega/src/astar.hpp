
#ifndef __AKAME_ASTAR

#define __AKAME_ASTAR

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>
#include <stdint.h>
#include <assert.h>

#include "std_msgs/msg/color_rgba.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "map.hpp"

struct timespec tnow(void) {
   struct timespec val;
   clock_gettime(CLOCK_REALTIME,&val);
   return val;
}

double timedist(struct timespec & t0, struct timespec & t1) {
   long long sec_dist = t1.tv_sec - t0.tv_sec;
   long long nsec_dist = t1.tv_nsec - t0.tv_nsec;

   return (double)sec_dist + (double)nsec_dist/1e9;
}

typedef struct node {
   int32_t x;
   int32_t y;
   int32_t idx;
   uint32_t f_score;
   uint32_t g_score;
   struct node * link;
} node;

typedef struct node_stub {
   int32_t x;
   int32_t y;
} node_stub;

struct node_comp_struct {
   bool operator()(const node * n1, const node * n2) {
      return n1->f_score > n2->f_score;
   }
} node_comp;

double dist2(double x0, double y0, double x1, double y1) {
   return ((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
}

class AStar {
public:

   /* for instantiation */
   AStar() {
      this->prev = std::unordered_map<uint64_t,node *>();
      this->queue = std::priority_queue<node *, std::vector<node *>, decltype(node_comp)>();
      this->queued = std::unordered_set<node *>();
      this->alloced = std::vector<node *>();
      this->path = std::vector<node_stub>();
      this->active = 0;
   }

   ~AStar() {
      cleanup();
   }

   int has_path() {
      if (path.size() == 0)
         return 0;
      return 1;
   }

   void route(double startx, double starty, double endx, double endy,
              double allotted, GridMap * map) {

      int32_t ex, ey, xlen, ylen;
      map->getidx2(endx,endy,ex,ey);
      map->lens(xlen,ylen);

      node * head;
      if (active) {
         head = pop();
      }
      else {
         cleanup();
         path.clear();

         int32_t sx, sy;
         map->getidx2(startx,starty,sx,sy);

         /* for some reason c++ doesn't support named
          * struct initialization even though it has been
          * supported in c since c99...                  */
         node new_head = {
            sx,
            sy,
            sx + sy * xlen,
            (uint32_t)manhattan(sx,sy,ex,ey),
            0, NULL,
         };
         head = allocate(new_head);
      }
      astar(head,ex,ey,allotted,map);
   }

   void reroute(double backup, double endx, double endy,
                double allotted, GridMap * map) {

      node * head;
      int32_t ex, ey;
      map->getidx2(endx,endy,ex,ey);

      if (active) {
         head = pop();
      }
      else {
         cleanup();

         uint32_t backstep = backup / map->res();

         node_stub * start_stub;
         if (backstep >= path.size()) {
            start_stub = &path[0];
            path.resize(1);
         }
         else {
            int start_pos = path.size()-backstep;
            for (int i = 1; i <= start_pos; ++i) {
               node_stub * entry = &path[i];
               if (map->is_obs(entry->x,entry->y,entry->x+entry->y*map->xlen)) {
                  start_stub = &path[i-1];
                  path.resize(i);
                  goto set_head;
               }
            }
            start_stub = &path[path.size()-backstep];
            path.resize(path.size()-backstep);
         }

set_head:
         node new_head = {
            start_stub->x,
            start_stub->y,
            start_stub->x + start_stub->y * map->xlen,
            0,
            (uint32_t)manhattan(start_stub->x,start_stub->y,ex,ey), 
            NULL,
         };
         head = allocate(new_head);
         assert(head && "something has gone terribly wrong...");
      }
      astar(head,ex,ey,allotted,map);
   }

   void clear() {
      active = 0;
      cleanup();
      path.clear();
   }

   // https://github.com/ros2/common_interfaces/blob/rolling/visualization_msgs/msg/Marker.msg
   void to_vis(visualization_msgs::msg::Marker & marker, GridMap * map) {
      marker.ns = "astar_vis";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
      marker.action = 0; /* add/modify */
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.x = 0;
      marker.scale.x = map->res();
      marker.scale.y = map->res();
      marker.scale.z = map->res();
      marker.lifetime.sec = 1;

      for (auto & entry : prev) {
         node * n = std::get<1>(entry);
         double x, y;
         map->getpos2(n->x,n->y,x,y);

         geometry_msgs::msg::Point p;
         p.x = x;
         p.y = y;
         p.z = 0.0;
         std_msgs::msg::ColorRGBA c;
         c.b = 0.0;
         c.a = 0.5;
         if (queued.count(n)) 
         { c.g = 0.0; c.r = 1.0; }
         else 
         { c.g = 1.0; c.r = 1.0; }

         marker.points.push_back(p);
         marker.colors.push_back(c);
      }
   }

   // https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Path.msg
   void to_msg(nav_msgs::msg::Path & msg, GridMap * map) {

      node_stub * pos = path.data();
      node_stub * end = pos + path.size();

      while (pos != end) {

         double x, y;
         map->getpos2(pos->x,pos->y,x,y);

         geometry_msgs::msg::PoseStamped pose;
         pose.pose.position.x = x;
         pose.pose.position.y = y;

         msg.poses.push_back(pose);

         pos += 1;
      }
   }


private:

#define EXTEND(x_step,y_step) \
do { \
   node left = { \
      curr->x + (x_step), \
      curr->y + (y_step), \
      curr->idx + (x_step) + (y_step)*xlen, \
      0, \
      curr->g_score + 1, NULL \
   }; \
   if ( ! map->is_obs(left.x,left.y,left.idx)) { \
      node * left_prev = previous(left); \
      if (left_prev) { \
         if (left.g_score < left_prev->g_score) { \
            left_prev->g_score = left.g_score; \
            left_prev->f_score = left.g_score  \
               + manhattan(left.x,left.y,ex,ey); \
            left_prev->link = curr; \
            insert(left_prev); \
         } \
      } \
      else { \
         left_prev = allocate(left); \
         left_prev->link = curr; \
         left_prev->f_score = left.g_score  \
            + manhattan(left.x,left.y,ex,ey); \
         insert(left_prev); \
         uint64_t key = left_prev->x | (((uint64_t)left_prev->y)<<32); \
         prev[key] = left_prev; \
      } \
   } \
} while (0)

   void astar(node * start, int32_t ex, int32_t ey,
              double allotted, GridMap * map) {

      struct timespec start_time = tnow();
      int32_t xlen, ylen;
      map->lens(xlen,ylen);

      insert(start);

      int steps = 0;
      while (!queue.empty()) {

         node * curr = pop();

         if (steps > 25) {
            steps = 0;
            struct timespec curr_time = tnow();
            if (timedist(curr_time,start_time) > allotted) {
               active = 1;
               goto end;
            }
         }

         if (curr->x == ex && curr->y == ey) {
            active = 0;
end:
            finish_path(curr);
            return;
         }

         EXTEND(-1,0); /* left */
         EXTEND(1,0); /* right */
         EXTEND(0,-1); /* up */
         EXTEND(0,1); /* down */

         steps += 1;
      }

      return;
   }

   inline void finish_path(node * curr) {
      std::vector<node_stub> buf;
      while (curr) {
         buf.push_back({curr->x,curr->y});
         curr = curr->link;
      }
      for (int i = buf.size()-1; i >= 0; --i)
         path.push_back(buf[i]);
   }

   inline node * find(node_stub & n) {
      uint64_t key = n.x | (((uint64_t)n.y)<<32);
      if (prev.count(key) > 0)
         return prev[key];
      return NULL;
   }

   inline node * previous(node & n) {
      uint64_t key = n.x | (((uint64_t)n.y)<<32);
      if (prev.count(key) > 0)
         return prev[key];
      return NULL;
   }

   inline node * pop() {
      node * val = queue.top();
      queue.pop();
      queued.erase(val);
      return val;
   }

   inline void insert(node * n) {
      if ( ! queued.count(n)) {
         queued.insert(n);
         queue.push(n);
      }
   }

   inline node * allocate(node & val) {
      node * new_node = (node *)malloc(sizeof(node));
      memcpy(new_node,&val,sizeof(node));

      alloced.push_back(new_node);

      return new_node;
   }

   inline int32_t manhattan(const int32_t sx, const int32_t sy, const int32_t ex, const int32_t ey) {
      return abs(sx - ex) + abs(sy - ey);
   }

   void empty_queue() {
      while (!queue.empty())
         queue.pop();
      queued.clear();
   }

   void cleanup() {
      node ** end = alloced.data() + alloced.size();
      node ** pos = alloced.data();
      while(pos < end) {
         free(*pos);
         pos += 1;
      }

      prev.clear();
      while (!queue.empty())
         queue.pop();
      queued.clear();
      alloced.clear();
      active = 0;
   }

   std::unordered_map<uint64_t,node *> prev;
   std::priority_queue<node *, std::vector<node *>, decltype(node_comp)> queue;
   std::unordered_set<node *> queued;
   std::vector<node *> alloced;
   std::vector<node_stub> path;
   int active;

};

#endif
