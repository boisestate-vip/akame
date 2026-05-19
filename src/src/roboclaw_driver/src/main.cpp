
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
#include <mutex>

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

      /* linear actuator → length-in-mm calibration, used to publish
       * /regolith/actuator_pos for the regolith_driver's mass model.
       *
       * calibration procedure:
       *   1. drive actuator to fully retracted, publish on calibrate_in
       *      so the M1 encoder gets reset to zero.
       *   2. measure pivot-to-pivot length with calipers → actuator_retracted_mm
       *   3. drive actuator to fully extended, read encoder → actuator_max_counts
       *   4. measure pivot-to-pivot length again → actuator_extended_mm
       *
       * defaults are placeholders; uncalibrated runs publish 0 + a warning. */
      this->declare_parameter("actuator_pos_out",   "/regolith/actuator_pos");
      this->declare_parameter("actuator_pub_hz",    20);
      this->declare_parameter("actuator_max_counts", 0);
      this->declare_parameter("actuator_retracted_mm", 0.0);
      this->declare_parameter("actuator_extended_mm",  0.0);

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

      actuator_pos_pub = this->create_publisher<std_msgs::msg::Float32>(
            this->get_parameter("actuator_pos_out").as_string(), 10);

      uint32_t baud = this->get_parameter("baud").as_int();
      printf("connecting to roboclaw on %s with baud %u\n",this->get_parameter("roboclaw_port").as_string().c_str(),baud);
      obj = new RoboClaw(this->get_parameter("roboclaw_port").as_string().c_str(),baud,10000);
      address = this->get_parameter("address").as_int();
      printf("got address as %d\n",address);

      in_pos_mode = 0;

      /* cache calibration so the publish path doesn't hit the parameter server every tick */
      actuator_max_counts   = this->get_parameter("actuator_max_counts").as_int();
      actuator_retracted_mm = this->get_parameter("actuator_retracted_mm").as_double();
      actuator_extended_mm  = this->get_parameter("actuator_extended_mm").as_double();
      if (actuator_max_counts <= 0) {
         printf("warning: actuator_max_counts <= 0 — /regolith/actuator_pos will publish 0 until calibrated\n");
      }

      int pub_hz = this->get_parameter("actuator_pub_hz").as_int();
      if (pub_hz <= 0) pub_hz = 20;
      actuator_timer = this->create_wall_timer(
            std::chrono::milliseconds(1000 / pub_hz),
            std::bind(&RoboclawDriver::publish_actuator_pos, this));

      homing_thread = std::thread(&RoboclawDriver::pos_worker,this);
      print_startup_info();
   }

private:

   RoboClaw * obj = NULL;
   int32_t address = 0x0;

   /* serializes all access to obj — the roboclaw is a single serial port,
    * and we have multiple threads (ROS executor + homing_thread) touching it. */
   std::mutex obj_mu;

   /* position flag and goal position in
    * meters above the ground plane.     */
   std::atomic<int> in_pos_mode;
   double pos_goal;
   int time_delay;
   std::thread homing_thread;

   /* linear actuator calibration (set from params at startup) */
   int    actuator_max_counts;
   double actuator_retracted_mm;
   double actuator_extended_mm;

   /* input ros node */
   rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_vel_in;
   rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_pos_in;
   rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr calibrate_trigger;

   /* output: linear actuator length in mm, consumed by regolith_driver */
   rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr actuator_pos_pub;
   rclcpp::TimerBase::SharedPtr actuator_timer;

   void collect_speed(const std_msgs::msg::Int32 & msg) {

      in_pos_mode = 0;
      std::lock_guard<std::mutex> lk(obj_mu);
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

               std::lock_guard<std::mutex> lk(obj_mu);
               obj->SpeedM1(address,dir*10000);
            }
            else {
               std::lock_guard<std::mutex> lk(obj_mu);
               obj->SpeedM1(address,0);
            }

         }
      }
   }

   void calibrate(const std_msgs::msg::Empty & msg) {
      (void)msg;
      in_pos_mode = 0;
      /* zero the M1 encoder so subsequent reads measure extension
       * from fully-retracted. user is expected to drive the actuator
       * to its hard stop before triggering this. */
      std::lock_guard<std::mutex> lk(obj_mu);
      obj->SetEncM1(address, 0);
   }

   /* read M1 encoder, convert to total pivot-to-pivot length in mm,
    * publish on /regolith/actuator_pos. */
   void publish_actuator_pos() {
      if (actuator_max_counts <= 0) {
         /* uncalibrated — publish 0 so the regolith_driver still sees the
          * topic and flips has_actuator_pos, but mass will be wrong. */
         auto msg = std_msgs::msg::Float32();
         msg.data = 0.0f;
         actuator_pos_pub->publish(msg);
         return;
      }

      uint8_t status = 0;
      bool valid = false;
      uint32_t enc;
      {
         std::lock_guard<std::mutex> lk(obj_mu);
         enc = obj->ReadEncM1(address, &status, &valid);
      }
      if (!valid) return;

      /* roboclaw encoders are unsigned 32-bit but wrap signed under the hood;
       * reinterpret as int32_t so retraction past zero reads negative. */
      int32_t enc_signed = (int32_t)enc;
      double fraction = (double)enc_signed / (double)actuator_max_counts;
      double length_mm = actuator_retracted_mm
                       + fraction * (actuator_extended_mm - actuator_retracted_mm);

      auto msg = std_msgs::msg::Float32();
      msg.data = (float)length_mm;
      actuator_pos_pub->publish(msg);
   }

   void print_startup_info() {
      char version_buf[128];
      memset(version_buf,0,sizeof(version_buf));

      std::lock_guard<std::mutex> lk(obj_mu);
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
