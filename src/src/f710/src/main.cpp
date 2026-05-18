
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

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "f710.h"

#define HARUNA_IS_BEST_GIRL 1

static void read_line(char * buf, int size) {
   if (fgets(buf,size-1,stdin) == NULL) buf[0] = '\0';
   int end = strlen(buf) - 1;
   while (isspace(buf[end]) && end > 0)
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

      this->declare_parameter("vel_out","/cmd_vel");
      this->declare_parameter("arm_out","/roboclaw_vel");
      this->declare_parameter("drum_out","/drum");

      /* maximum velocity to scale everything by */
      this->declare_parameter("max_vel", 1.0);

      /* maximum arm speed */
      this->declare_parameter("max_arm_speed", 10000);

      /* maximum drum speed */
      this->declare_parameter("max_drum_speed", 15.0);

      /* the distance between the tracks */
      this->declare_parameter("track_spacing", 1.0);

      /* the device to connect to the controller on */
      this->declare_parameter("device","/dev/hidraw1");

      /* whether to ask the user for the above paramters instead */
      this->declare_parameter("interactive",true);

      /* interval in seconds to publish messages
       * Setting this higher will result in a smoother
       * control experience                           */
      this->declare_parameter("publish_rate", 0.05);

      cmd_out = this->create_publisher<geometry_msgs::msg::Twist>(
            this->get_parameter("vel_out").as_string(), 10);
      arm_out = this->create_publisher<std_msgs::msg::Int32>(
            this->get_parameter("arm_out").as_string(), 10);
      drum_out = this->create_publisher<std_msgs::msg::Float64>(
            this->get_parameter("drum_out").as_string(), 10);

      publish_rate = this->get_parameter("publish_rate").as_double();

      const char * device;

      char namebuf[128], velbuf[128], wheelbuf[128];
      std::string device_str = this->get_parameter("device").as_string();
      if (this->get_parameter("interactive").as_bool()) {

         fprintf(stderr,"enter the device to connect to: ");
         read_line(namebuf,128);
         device = namebuf;

         fprintf(stderr,"enter the max velocity for control: ");
         read_line(velbuf,128);
         max_vel = strtod(velbuf,NULL);

         fprintf(stderr,"enter the track spacing (blank for default): ");
         read_line(wheelbuf,128);
         if (strlen(wheelbuf) == 0) {
            fprintf(stderr,"using default track spacing of 1.0\n");
            track_spacing = 1.0;
         }
         else {
            track_spacing = strtod(wheelbuf,NULL);
         }
         if (track_spacing <= 0.01) {
            fprintf(stderr, "track spacing is too small (<=0.01). Using 0.01 as a default.\n");
            track_spacing = 0.01;
         }
      }
      else {
         device = device_str.c_str();
         max_vel = this->get_parameter("max_vel").as_double();
         track_spacing = this->get_parameter("track_spacing").as_double();
      }

      max_arm_speed = this->get_parameter("max_arm_speed").as_int();
      arm_speed_mult = 1.0;
      
      max_drum_speed = this->get_parameter("max_drum_speed").as_double();
      drum_speed_mult = 1.0;

      last_arm_adjust = std::chrono::steady_clock::now();
      last_drum_adjust = std::chrono::steady_clock::now();

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

   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_out;
   rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_out;
   rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr drum_out;

   /* callback interval to publish twist */
   rclcpp::TimerBase::SharedPtr cmd_callback;

   /* up to date twist to publish from f710 thread */
   geometry_msgs::msg::Twist last_twist;
   std_msgs::msg::Int32 last_arm;
   std_msgs::msg::Float64 last_drum;
   std::mutex state_lock;

   std::thread f710_thread;

   /* the connection we are listening on */
   int fd;

   /* velocity multiplier */
   double max_vel;

   /* the distance between the tracks */
   double track_spacing;

   /* arm speed variables */
   int max_arm_speed;
   double arm_speed_mult;
   std::chrono::steady_clock::time_point last_arm_adjust;

   /* drum speed variables */
   double max_drum_speed;
   double drum_speed_mult;
   std::chrono::steady_clock::time_point last_drum_adjust;

   /* publish rate in seconds */
   double publish_rate;

   void publish_cmd_vel() {
      state_lock.lock();
         cmd_out->publish(last_twist);
         arm_out->publish(last_arm);
         drum_out->publish(last_drum);
      state_lock.unlock();
   }

   void get_f710_values() {

      struct f710_status stat;

      /* we do all our processing in the initializer... */
      while (HARUNA_IS_BEST_GIRL) {

         if (f710_read_next(fd,&stat)) {

            double lwh = ((double)(stat.lv_fr - 127) / -128.0) * max_vel;
            double rwh = ((double)(stat.rv_fr - 127) / -128.0) * max_vel;

            /* Allow for scaling the arm and drum speeds by holding down the mode button.
             * This is useful for fine control or testing different speeds manually.
             * Use the A/B buttons to adjust the drum speed multiplier and the X/Y buttons
             * to adjust the arm speed multiplier. Speeds are constrained between 0.0 and 1.5x
             * to prevent reversing direction or excessive speeds.
             * 
             * Updates are rate limited to 5 Hz to prevent excessive adjustments from a single button press.
             * */
            if (stat.mode) {
               auto now = std::chrono::steady_clock::now();
               auto arm_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_arm_adjust).count();
               auto drum_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_drum_adjust).count();

               if (drum_elapsed > 200) {
                  if (stat.a) {
                     drum_speed_mult = fmax(drum_speed_mult - 0.05, 0.0);
                     last_drum_adjust = now;
                  } else if (stat.b) {
                     drum_speed_mult = fmin(drum_speed_mult + 0.05, 1.5);
                     last_drum_adjust = now;
                  }
               }
               

               if (arm_elapsed > 200) {
                  if (stat.x) {
                     arm_speed_mult = fmax(arm_speed_mult - 0.05, 0.0);
                     last_arm_adjust = now;
                  } else if (stat.y) {
                     arm_speed_mult = fmin(arm_speed_mult + 0.05, 1.5);
                     last_arm_adjust = now;
                  }
               }
            }

            // Move the arm up/down using the left trigger and bumper
            int32_t arm_speed = 0;
            if (stat.lt)
               arm_speed = (int)(-max_arm_speed * arm_speed_mult);
            else if (stat.lb)
               arm_speed = (int)(max_arm_speed * arm_speed_mult);

            // Move the drum fw/back using the right trigger and bumper
            double drum_speed = 0;
            if (stat.rt)
               drum_speed = -max_drum_speed * drum_speed_mult;
            else if (stat.rb)
               drum_speed = max_drum_speed * drum_speed_mult;

            printf("lwh: %lf, rwh: %lf, arm: %d, drum: %lf\n",
                   lwh,rwh,arm_speed,drum_speed);

            /* here we compute the angular velocity. 
             * This seems logical to me, given that the controller 
             * is kind of an abstract vehicle in the sense of a 
             * differential drive system anyway...
             * */
            // https://en.wikipedia.org/wiki/Differential_wheeled_robot

            double b = track_spacing;
            double w = (rwh - lwh) / b;
            double V = (rwh + lwh) / 2.0;

            state_lock.lock();
               last_twist.linear.x = V;
               last_twist.angular.z = w;
               last_arm.data = arm_speed;
               last_drum.data = drum_speed;
            state_lock.unlock();
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
