
/* a simple driver for the waveshare robot that takes in a twist
 * message and publishes it over usb connection to the robot
 *
 * intended to be used by a raspberry pi mounted directly on
 * the robot.                                                  */

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"

#include <functional>

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

      /* damping values */
      this->declare_parameter("cmd_in","/roboclaw");

      /* motor address */
      this->declare_parameter("address",0x80);


      /* setup variables */
      cmd_in = this->create_subscription<std_msgs::msg::Int32>(
            this->get_parameter("cmd_in").as_string(), 10,
            std::bind(&RoboclawDriver::collect_speed, this, _1));

      const char * port = this->get_parameter("roboclaw_port").as_string().c_str();
      uint32_t baud = this->get_parameter("baud").as_int();
      printf("connecting to roboclaw on %s with baud %u\n",port,baud);
      obj = new RoboClaw(port,baud,10000);
      address = this->get_parameter("address").as_int();
      printf("got address as %d\n",address);

      print_startup_info();
   }

private:

   RoboClaw * obj = NULL;
   int32_t address = 0x0;

   /* input ros node */
   rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_in;

   void collect_speed(const std_msgs::msg::Int32 & msg) {

      obj->SpeedM1(address,msg.data);
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
