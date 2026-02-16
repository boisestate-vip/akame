
#ifndef __AKAME_POINTS

#define __AKAME_POINTS
#include <vector>

struct point2 {
   double x;
   double y;
};

class Points {
public:

   double xcenter;
   double ycenter;
   double xmin;
   double ymin;
   double xmax;
   double ymax;

   std::vector<point2> points;

   Points(double xcenter, double ycenter) : xcenter(xcenter), ycenter(ycenter) {}

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
