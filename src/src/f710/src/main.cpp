
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

#include <thread>
#include <mutex>

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

class F710 : public rclcpp::Node {
public:

   F710() : Node("f710") {

      /* the topic to listen for scans on */
      this->declare_parameter("vel_out","cmd_vel");

      /* maximum velocity to scale everything by */
      this->declare_parameter("max_vel", 1.0);

      /* the device to connect to the controller on */
      this->declare_parameter("device","/dev/hidraw1");

      /* whether to ask the user for the above paramters instead */
      this->declare_parameter("interactive",false);

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
         std::string device_str = this->get_parameter("device").as_string();
         device = device_str.c_str();
         max_vel = this->get_parameter("max_vel").as_double();
      }

      RCLCPP_INFO(this->get_logger(),"got device %s and max_vel %f",device,max_vel);

      fd = try_connect(device);
      if (fd < 0) {
         RCLCPP_ERROR(this->get_logger(),"failed connecting to device %s",device);
         exit(1);
      }

      last_twist.linear.x = 0; last_twist.linear.y = 0;
      last_twist.linear.z = 0; last_twist.angular.x = 0;
      last_twist.angular.y = 0; last_twist.angular.z = 0;

      cmd_callback = this->create_wall_timer(
            std::chrono::milliseconds((long)(1000.0 *
                  this->get_parameter("publish_rate").as_double())),
            std::bind(&F710::publish_cmd_vel, this));

      f710_thread = std::thread(&F710::get_f710_values, this);
   }

private:

   /* reception area for the scans */
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_out;

   /* callback interval to publish twist */
   rclcpp::TimerBase::SharedPtr cmd_callback;

   /* up to date twist to publish from f710 thread */
   geometry_msgs::msg::Twist last_twist;
   std::mutex twist_lock;

   std::thread f710_thread;

   /* the connection we are listening on */
   int fd;

   /* velocity multiplier */
   double max_vel;

   /* publish rate in seconds */
   double publish_rate;

   void publish_cmd_vel() {
      twist_lock.lock();
         cmd_out->publish(last_twist);
      twist_lock.unlock();
   }

   void get_f710_values() {

      struct f710_status stat;

      /* we do all our processing in the initializer... */
      while (HARUNA_IS_BEST_GIRL) {

         if (f710_read_next(fd,&stat)) {

            double lwh = ((double)(stat.lv_fr - 127) / -128.0) * max_vel;
            double rwh = ((double)(stat.rv_fr - 127) / -128.0) * max_vel;

            /* here we compute the angular velocity assuming
             * a value of 1.0 for b. This seems logical to
             * me, given that the controller is kind of an
             * abstract vehicle in the sense of a differential
             * drive system anyway...                         */
            // https://en.wikipedia.org/wiki/Differential_wheeled_robot

            double w = rwh - lwh;
            double V = (rwh + lwh) / 2.0;

            twist_lock.lock();
               last_twist.linear.x = V;
               last_twist.angular.z = w;
            twist_lock.unlock();
         }
      }
   }

};

int main(int argc, char ** argv) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<F710>());
   rclcpp::shutdown();
   return 0;
}
