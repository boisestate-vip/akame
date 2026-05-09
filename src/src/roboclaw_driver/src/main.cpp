
/* a simple driver for the waveshare robot that takes in a twist
 * message and publishes it over usb connection to the robot
 *
 * intended to be used by a raspberry pi mounted directly on
 * the robot.                                                  */

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/empty.hpp"

#include <functional>
#include <atomic>
#include <thread>

#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>

#include "roboclaw.hpp"

using std::placeholders::_1;

class RoboclawDriver : public rclcpp::Node {
public:

   RoboclawDriver() : Node("roboclaw_driver") {


      /* the devices the left and right motors are 
       * connected on.                             */
      this->declare_parameter("roboclaw_port","/dev/ttyACM0");

      /* baud rate to run at */
      this->declare_parameter("baud",34800);

      /* topics to subscribe on */
      this->declare_parameter("cmd_vel_in","/roboclaw_vel");
      this->declare_parameter("cmd_pos_in","/roboclaw_pos");
      this->declare_parameter("calibrate_in","/roboclaw_calibrate");

      /* motor address */
      this->declare_parameter("address",0x80);

      /* Hz to run the position homing callback at */
      this->declare_parameter("pos_homing_hz",50);
      time_delay = 1000000 / this->get_parameter("pos_homing_hz").as_int();


      /* setup variables */
      cmd_vel_in = this->create_subscription<std_msgs::msg::Int32>(
            this->get_parameter("cmd_vel_in").as_string(), 10,
            std::bind(&RoboclawDriver::collect_speed, this, _1));

      cmd_pos_in = this->create_subscription<std_msgs::msg::Float32>(
            this->get_parameter("cmd_pos_in").as_string(), 10,
            std::bind(&RoboclawDriver::collect_pos, this, _1));

      calibrate_trigger = this->create_subscription<std_msgs::msg::Empty>(
            this->get_parameter("calibrate_in").as_string(), 10,
            std::bind(&RoboclawDriver::calibrate, this, _1));

      uint32_t baud = this->get_parameter("baud").as_int();
      printf("connecting to roboclaw on %s with baud %u\n",this->get_parameter("roboclaw_port").as_string().c_str(),baud);
      obj = new RoboClaw(this->get_parameter("roboclaw_port").as_string().c_str(),baud,10000);
      address = this->get_parameter("address").as_int();
      printf("got address as %d\n",address);

      in_pos_mode = 0;

      homing_thread = std::thread(&RoboclawDriver::pos_worker,this);
      print_startup_info();
   }

private:

   RoboClaw * obj = NULL;
   int32_t address = 0x0;

   /* position flag and goal position in
    * meters above the ground plane.     */
   std::atomic<int> in_pos_mode;
   double pos_goal;
   int time_delay;
   std::thread homing_thread;

   /* input ros node */
   rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_vel_in;
   rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_pos_in;
   rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr calibrate_trigger;

   void collect_speed(const std_msgs::msg::Int32 & msg) {

      in_pos_mode = 0;
      obj->SpeedM1(address,msg.data);
   }

   void collect_pos(const std_msgs::msg::Float32 & msg) {
      pos_goal = msg.data;
      in_pos_mode = 1;
   }

   void pos_worker() {

      for (;;) {
         usleep(time_delay);

         if (in_pos_mode) {

            // get the position somehow
            double pos_curr = 0.0;

            if (std::fabs(pos_curr - pos_goal) > 0.01) {
               int dir = pos_goal - pos_curr > 0 ? 1 : -1;

               obj->SpeedM1(address,dir*10000);
            }
            else {
               obj->SpeedM1(address,0);
            }

         }
      }
   }

   void calibrate(const std_msgs::msg::Empty & msg) {
      (void)msg;
      in_pos_mode = 0;
      // may impliment this as a service...
   }

   void print_startup_info() {
      char version_buf[128];
      memset(version_buf,0,sizeof(version_buf));

      obj->ReadVersion(address,version_buf);
      printf("Roboclaw Version: %s\n",version_buf);

      float kp_fp = 0, ki_fp = 0, kd_fp = 0;
      uint32_t qpps = 0;
      obj->ReadM1VelocityPID(address,kp_fp,ki_fp,kd_fp,qpps);
      printf("kp_fp: %f\n",kp_fp);
      printf("ki_fp: %f\n",kp_fp);
      printf("kd_fp: %f\n",kp_fp);
      printf("qpps: %d\n",qpps);
   }

};

int main(int argc, char ** argv) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<RoboclawDriver>());
   rclcpp::shutdown();
   return 0;
}
