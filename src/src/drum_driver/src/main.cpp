
/* a simple driver for the waveshare robot that takes in a twist
 * message and publishes it over usb connection to the robot
 *
 * intended to be used by a raspberry pi mounted directly on
 * the robot.                                                  */

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"

#include <functional>

#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

using std::placeholders::_1;

class DrumDriver : public rclcpp::Node {
public:

   DrumDriver() : Node("go_m8010_6_driver") {

      /* incoming command inputs */
      this->declare_parameter("cmd_in","/drum");

      /* the devices the motor is connected on */
      this->declare_parameter("motor","/dev/ttyUSB4");

      /* the id of the device. These are
       * settable using the ./configid script from
       * https://github.com/unitreerobotics/unitree_actuator_sdk/tree/main/motor_tools
       */
      this->declare_parameter("motor_id",2);

      /* damping values */
      this->declare_parameter("position_damping",0.005);
      this->declare_parameter("velocity_damping",0.01);

      /* setup variables */
      cmd_in = this->create_subscription<std_msgs::msg::Float64>(
            this->get_parameter("cmd_in").as_string(), 10,
            std::bind(&DrumDriver::collect_vel, this, _1));

      /* last wheel speeds */
      last_wh = 0.0;

      /* save wheel ids */
      id = this->get_parameter("motor_id").as_int();

      /* setup our connections */
      std::string serial_device = this->get_parameter("motor").as_string();
      RCLCPP_INFO(this->get_logger(),"connecting to device %s",
                  serial_device.c_str());
      try {
         serial = new SerialPort(serial_device);
      } 
      catch (IOException &) {
         RCLCPP_ERROR(this->get_logger(),"caught io exception connecting to %s, exiting",serial_device.c_str());
         perror("errno");
         exit(1);
      }

      /* save motor constants */
      kp = this->get_parameter("position_damping").as_double();
      kd = this->get_parameter("velocity_damping").as_double();
   }

private:

   /* class variables */

   /* constant for motor computation */
   double kp;
   double kd;

   /* motor ids */
   int id;

   /* serial devices we are interacting with the 8010 on */
   SerialPort * serial;

   /* input ros node */
   rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_in;

   /* periodic transmission values */
   double last_wh;

   void collect_vel(const std_msgs::msg::Float64 & msg) {

      write_value(msg.data);
      last_wh = msg.data;
   }

   /* taken from:
    * https://github.com/unitreerobotics/unitree_actuator_sdk
    */
   void write_value(double wh) {
      MotorCmd cmd;
      MotorData data;

      double gearRatio = queryGearRatio(MotorType::GO_M8010_6);

      cmd.motorType = MotorType::GO_M8010_6;
      data.motorType = MotorType::GO_M8010_6;
      cmd.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
      cmd.id = id;
      cmd.kp = 0.0; /* we aren't using position right now */
      cmd.kd = kd;
      cmd.q = 0.0;
      cmd.dq = wh * gearRatio;
      cmd.tau = 0.0;

      try {
         serial->sendRecv(&cmd,&data);
      } catch (IOException &) {
         perror("error");
      }
   }
};

int main(int argc, char ** argv) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<DrumDriver>());
   rclcpp::shutdown();
   return 0;
}
