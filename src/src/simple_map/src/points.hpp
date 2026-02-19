
#ifndef __AKAME_POINTS

#define __AKAME_POINTS
#include <vector>
#include <cfloat>

struct point2 {
   double x;
   double y;
   int has_obstacle;
};

class Points {
public:

   double xcenter;
   double ycenter;
   double xmin = FLT_MAX; /* flt max/min are both positive */
   double ymin = FLT_MAX;
   double xmax = -FLT_MAX;
   double ymax = -FLT_MAX;

   std::vector<point2> points;

   Points(double xcenter, double ycenter) : xcenter(xcenter), ycenter(ycenter) {
      points = std::vector<point2>();
   }

   void add_point(point2 p) {

      if (p.x > xmax)
         xmax = p.x;
      else if (p.x < xmin)
         xmin = p.x;

      if (p.y > ymax)
         ymax = p.y;
      else if (p.y < ymin)
         ymin = p.y;

      points.push_back(p);
   }
};

#endif
