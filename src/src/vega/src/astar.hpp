
#ifndef __AKAME_ASTAR

#define __AKAME_ASTAR

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>
#include <stdint.h>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "map.hpp"

typedef struct node {
   int32_t x;
   int32_t y;
   int32_t idx;
   uint32_t score;
   struct node * link;
} node;

struct {
   bool operator()(const node * n1, const node * n2) {
      return n1->score > n2->score;
   }
} node_comp;

double dist2(double x0, double y0, double x1, double y1) {
   return ((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
}

class AStar {
public:

   /* for instantiation */
   AStar() {
      this->prev = std::unordered_map<uint32_t,node *>();
      this->queue = std::priority_queue<node *, std::vector<node *>, decltype(node_comp)>();
      this->queued = std::unordered_set<node *>();
      this->alloced = std::vector<node *>();
      this->root = NULL;
      this->active = 0;
   }

   ~AStar() {
      cleanup();
   }

   void route(double startx, double starty, double endx, double endy,
              clock_t allotted, GridMap * map) {

      int32_t ex, ey;
      map->getidx2(endx,endy,ex,ey);

      node * head;
      if (active) {
         head = pop();
      }
      else {
         cleanup();

         int32_t sx, sy, sidx;
         map->getidx2(startx,starty,sx,sy);
         sidx = sx + sy * map->xlen;

         head = alloc_node(NULL,sx,sy,sidx);
         head->score = 0;
      }
      root = astar(head,ex,ey,allotted,map);
   }

   void reroute(double backup, double endx, double endy,
                clock_t allotted, GridMap * map) {

      node * head;
      int32_t ex, ey;
      map->getidx2(endx,endy,ex,ey);

      if (active) {
         head = pop();
      }
      else {
         empty_queue();

         uint32_t backstep = backup / map->res();
      }
      root = astar(head,ex,ey,allotted,map);
   }

   void clear() {
      active = 0;
      cleanup();
   }

   void to_msg(nav_msgs::msg::Path const & msg, GridMap * map) {

      node * curr = root;

      while (curr) {

         double x, y;
         map->getpos2(curr->x,curr->y,x,y);

         geometry_msgs::msg::PoseStamped pose;
         pose.pose.position.x = x;
         pose.pose.position.y = y;

         msg.poses.push_back(pose);
      }
   }


private:

   node * astar(node * start, int32_t ex, int32_t ey,
                clock_t allotted, GridMap * map) {


      while (!queue.empty()) {

         node * curr = queue.top();
         queue.pop();

         if (curr->x == ex && curr->y == ey) {
            node * prev = curr, * next;
            curr = curr->link;
            while (curr) {

               next = curr->link;
               curr->link = prev;
               prev = curr;
               curr = next;

            }
            return prev;
         }

      }

      return NULL;
   }

   inline node * pop() {
      node * val = queue.top();
      queue.pop();
      queued.erase(val);
   }

   inline void put(node * val, node * parent, int32_t score) {

      node * curr = prev[val->idx];
      if (curr) {
         if (score < curr->score) {
            val->score = score;
            val->link = parent;
            prev[val->idx] = val;
         }
      }
      else {
         val->link = parent;
         val->score = score;
      }
   }

   inline const int32_t manhattan(const int32_t sx, const int32_t sy, const int32_t ex, const int32_t ey) {
      return abs(sx - ex) + abs(sy - ey);
   }

   node * alloc_node(node * parent, int32_t x, int32_t y, int32_t idx) {
      node * new_node = (node *)malloc(sizeof(node));
      memset(new_node,0,sizeof(node));
      new_node->link = parent;

      new_node->x = x;
      new_node->y = y;
      new_node->idx = idx;

      alloced.push_back(new_node);

      return new_node;
   }

   void empty_queue() {
      while (!queue.empty())
         queue.pop();
   }

   void cleanup() {
      node ** end = alloced.data() + alloced.size();
      node ** pos = alloced.data();
      while(pos < end)
         free(*pos);

      root = NULL;
      prev.clear();
      while (!queue.empty())
         queue.pop();
      alloced.clear();
   }

   std::unordered_map<uint32_t,node *> prev;
   std::priority_queue<node *, std::vector<node *>, decltype(node_comp)> queue;
   std::unordered_set<node *> queued;
   std::vector<node *> alloced;
   node * root;
   int active;

};

#endif
