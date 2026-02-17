
#ifndef __AKAME_F710

#define __AKAME_F710
#include <cstdint>

/* most of this was lifted straight from
 * the f710 stuff in bender-2025         */

struct f710_status {
   int8_t on;
   int8_t vibrate;
   int8_t mode;
   int8_t start;
   int8_t back;
   int8_t dir;
   int8_t y;
   int8_t x;
   int8_t a;
   int8_t b;
   int8_t lt;
   int8_t lb;
   int8_t rt;
   int8_t rb;
   uint8_t lv_fr;
   uint8_t lv_lr;
   uint8_t rv_fr;
   uint8_t rv_lr;
};

#define POLL_RATE 0.125

#define DIR_CENTER 8
#define DIR_NORTH 0
#define DIR_SOUTH 4
#define DIR_WEST 6
#define DIR_EAST 2
#define DIR_SOUTHWEST 5
#define DIR_SOUTHEAST 3
#define DIR_NORTHEAST 1
#define DIR_NORTHWEST 7

#define CONTROL_ON 0x1000000000000000
#define VIBRATE_ON 0x2000000000000000
#define MODE_ON    0x0800000000000000
#define Y_DOWN     0x0000800000000000
#define X_DOWN     0x0000100000000000
#define A_DOWN     0x0000200000000000
#define B_DOWN     0x0000400000000000
#define LT_DOWN    0x0004000000000000
#define LB_DOWN    0x0001000000000000
#define RT_DOWN    0x0008000000000000
#define RB_DOWN    0x0002000000000000
#define START_DOWN 0x0020000000000000
#define BACK_DOWN  0x0010000000000000

#define PRESSED 1
#define RELEASED 0

#define UPDATE_FLAG(og,stat,flag,val) \
   if (og & val)                      \
      stat->flag = PRESSED;           \
   else                               \
      stat->flag = RELEASED

void f710_update_status(long og, struct f710_status * stat);

/* return true if a new value was grabbed within
 * the timeframe, false otherwise               */
int f710_read_next(int fd, struct f710_status * stat);


#endif
