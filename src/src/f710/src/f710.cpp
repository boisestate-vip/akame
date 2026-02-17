
#include "f710.h"
#include <unistd.h>
#include <string.h>
#include <sys/select.h>

void f710_update_status(long og, struct f710_status * stat) {

   UPDATE_FLAG(og,stat,vibrate,VIBRATE_ON);
   UPDATE_FLAG(og,stat,on,CONTROL_ON);
   UPDATE_FLAG(og,stat,mode,MODE_ON);
   UPDATE_FLAG(og,stat,start,START_DOWN);
   UPDATE_FLAG(og,stat,back,BACK_DOWN);
   UPDATE_FLAG(og,stat,y,Y_DOWN);
   UPDATE_FLAG(og,stat,x,X_DOWN);
   UPDATE_FLAG(og,stat,a,A_DOWN);
   UPDATE_FLAG(og,stat,b,B_DOWN);
   UPDATE_FLAG(og,stat,lt,LT_DOWN);
   UPDATE_FLAG(og,stat,lb,LB_DOWN);
   UPDATE_FLAG(og,stat,rt,RT_DOWN);
   UPDATE_FLAG(og,stat,rb,RB_DOWN);

   stat->dir   = (og >> 40) & 0xf;
   stat->lv_fr = (og >> 16) & 0xff;
   stat->lv_lr = (og >>  8) & 0xff;
   stat->rv_fr = (og >> 32) & 0xff;
   stat->rv_lr = (og >> 24) & 0xff;

}

int f710_read_next(int fd, struct f710_status * stat) {

   struct timeval wait_max;
   fd_set read_set;

   FD_ZERO(&read_set);
   FD_SET(fd,&read_set);
   bzero(&wait_max,sizeof(wait_max));
   wait_max.tv_sec = 0;
   wait_max.tv_usec = (int)(POLL_RATE * 1e6);

   int res = select(fd+1,&read_set,NULL,NULL,&wait_max);
   long recved;

   if (res > 0) {

      read(fd,&recved,sizeof(long));

      f710_update_status(recved,stat);

      return 1;
   }
   return 0;
}
