
/* a small driver that wraps the f710 over
 * usb connection and publishes messages
 *
 * the controller does not consistantly
 * bind to a /dev address, so the user is
 * prompted to add it when launching the
 * program if the flag is set. This makes
 * it a bit difficult to do scripts, but I
 * felt it was the best way to go about things.
 */

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <time.h>

#include "f710.h"

#define HARUNA_IS_BEST_GIRL 1

static void read_line(char * buf, int size) {
   fgets(buf,size-1,stdin);
   int end = strlen(buf) - 1;
   while (isspace(buf[end]))
      end -= 1;
   buf[end+1] = '\0';
}

static int try_connect(const char * device) {

   int fd = open(device,O_RDONLY);
   if (fd < 0) {
      perror("open");
      return -1;
   }

   return fd;
}

/* artifact of f710_cli_new :) */
#define NANOSECONDS_PER_SECOND 1000000000
clock_t CLOCK() {
   struct timespec uptime;
   clock_gettime(CLOCK_MONOTONIC,&uptime);
   return 1e9 * uptime.tv_sec + uptime.tv_nsec;
}

class F710 : public rclcpp::Node {
public:

   F710() : Node("f710") {

      /* the topic to listen for scans on */
      this->declare_parameter("vel_out","cmd_vel");

      /* maximum velocity to scale everything by */
      this->declare_parameter("max_vel", 3.0);

      /* the device to connect to the controller on */
      this->declare_parameter("device","/dev/hidraw0");

      /* whether to ask the user for the above paramters instead */
      this->declare_parameter("interactive",true);

      /* interval in seconds to publish messages
       * Setting this higher will result in a smoother
       * control experience                           */
      this->declare_parameter("publish_rate", 0.05);

      cmd_out = this->create_publisher<geometry_msgs::msg::Twist>(
            this->get_parameter("vel_out").as_string(), 10);

      publish_rate = this->get_parameter("publish_rate").as_double();

      const char * device;

      char namebuf[128], velbuf[128];
      if (this->get_parameter("interactive").as_bool()) {

         fprintf(stderr,"enter the device to connect to: ");
         read_line(namebuf,128);
         device = namebuf;

         fprintf(stderr,"enter the max velocity for control: ");
         read_line(velbuf,128);
         max_vel = strtod(velbuf,NULL);
      }
      else {
         device = this->get_parameter("device").as_string().c_str();
         max_vel = this->get_parameter("max_vel").as_double();
      }

      RCLCPP_INFO(this->get_logger(),"got device %s and max_vel %f",device,max_vel);

      fd = try_connect(device);
      if (fd < 0) {
         RCLCPP_ERROR(this->get_logger(),"failed connecting to device %s",device);
         exit(1);
      }

      geometry_msgs::msg::Twist msg;
      msg.linear.x = 0; msg.linear.y = 0;
      msg.linear.z = 0; msg.angular.x = 0;
      msg.angular.y = 0; msg.angular.z = 0;
      struct f710_status stat;
      uint64_t last_clock = 0;

      /* we do all our processing in the initializer... */
      while (HARUNA_IS_BEST_GIRL) {

         
         uint64_t raw_dist = CLOCK() - last_clock;
         double time_dist = (double)(raw_dist) / NANOSECONDS_PER_SECOND;
         if (time_dist > publish_rate) {
            last_clock = CLOCK();
         }
         else {
            /* wait the given time period */
            struct timespec waiter;
            waiter.tv_sec = raw_dist / NANOSECONDS_PER_SECOND;
            waiter.tv_nsec = raw_dist % NANOSECONDS_PER_SECOND;
            nanosleep(&waiter,NULL);
            continue;
         }

         if (f710_read_next(fd,&stat)) {

            double lwh = ((double)(stat.lv_fr - 127) / 128.0) * max_vel;
            double rwh = ((double)(stat.rv_fr - 127) / 128.0) * max_vel;

            /* here we compute the angular velocity assuming
             * a value of 1.0 for b. This seems logical to
             * me, given that the controller is kind of an
             * abstract vehicle in the sense of a differential
             * drive system anyway...                         */
            // https://en.wikipedia.org/wiki/Differential_wheeled_robot

            double w = rwh - lwh;
            double V = (rwh + lwh) / 2.0;

            msg.linear.x = V;
            msg.angular.z = w;

            cmd_out->publish(msg);
         }
      }
   }

private:

   /* reception area for the scans */
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_out;

   /* the connection we are listening on */
   int fd;

   /* velocity multiplier */
   double max_vel;

   /* publish rate in seconds */
   double publish_rate;

};

int main(int argc, char ** argv) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<F710>());
   rclcpp::shutdown();
   return 0;
}
