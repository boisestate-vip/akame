
#ifndef __COMMON

#define __COMMON

typedef struct point {
   double x;
   double y;
} point;

typedef struct point_val {
   point p;
   double val;
} point_val;

typedef struct node {
   struct node * prev;
   struct node * next;
   point pos;
   double d;
   double n;
} node;

#endif
