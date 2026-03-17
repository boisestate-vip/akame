
/* a simple driver for the waveshare robot that takes in a twist
 * message and publishes it over usb connection to the robot
 *
 * intended to be used by a raspberry pi mounted directly on
 * the robot.                                                  */

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

#include <functional>

#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

using std::placeholders::_1;

class Go8010Driver : public rclcpp::Node {
public:

   Go8010Driver() : Node("go_m8010_6_driver") {

      /* incoming command inputs */
      this->declare_parameter("cmd_in","cmd_vel");

      /* the devices the left and right motors are 
       * connected on.                             */
      this->declare_parameter("left_motor","/dev/ttyUSB0");
      this->declare_parameter("right_motor","/dev/ttyUSB1");

      /* the ids of the left and right devices. These are
       * settable using the ./configid script from
       * https://github.com/unitreerobotics/unitree_actuator_sdk/tree/main/motor_tools
       */
      this->declare_parameter("left_motor_id",0);
      this->declare_parameter("right_motor_id",1);

      /* damping values */
      this->declare_parameter("position_damping",0.005);
      this->declare_parameter("velocity_damping",0.01);

      /* the radius of the wheels for the differential drive calculation (meters) */
      //this->declare_parameter("wheel_radius",0.037);
      this->declare_parameter("wheel_radius",1.0);

      /* the width of the vehicle for the differential drive calculation (meters) */
      this->declare_parameter("vehicle_width",0.076);

      /* setup variables */
      cmd_in = this->create_subscription<geometry_msgs::msg::Twist>(
            this->get_parameter("cmd_in").as_string(), 10,
            std::bind(&Go8010Driver::collect_cmd_vel, this, _1));

      /* save the diff-drive values */
      b2 = this->get_parameter("wheel_radius").as_double() / 2.0;
      r = this->get_parameter("vehicle_width").as_double();

      /* last wheel speeds */
      last_lwh = 0.0;
      last_rwh = 0.0;

      /* save wheel ids */
      left_id = this->get_parameter("left_motor_id").as_int();
      right_id = this->get_parameter("right_motor_id").as_int();

      /* setup our connections */
      std::string serial_l_device = this->get_parameter("left_motor").as_string();
      std::string serial_r_device = this->get_parameter("right_motor").as_string();
      RCLCPP_INFO(this->get_logger(),"connecting to device %s",
                  serial_l_device.c_str());
      try {
         serial_l = new SerialPort(serial_l_device);
      } 
      catch (IOException &) {
         RCLCPP_ERROR(this->get_logger(),"caught io exception connecting to %s, exiting",serial_l_device.c_str());
         perror("errno");
         exit(1);
      }
      RCLCPP_INFO(this->get_logger(),"connecting to device %s",
                  serial_r_device.c_str());
      try {
         serial_r = new SerialPort(serial_r_device);
      } 
      catch (IOException &) {
         RCLCPP_ERROR(this->get_logger(),"caught io exception connecting to %s, exiting",serial_r_device.c_str());
         perror("errno");
         exit(1);
      }


      /* save motor constants */
      kp = this->get_parameter("position_damping").as_double();
      kd = this->get_parameter("velocity_damping").as_double();
   }

private:

   /* class variables */

   /* constants for the diff-drive computation */
   double b2;
   double r;

   /* constant for motor computation */
   double kp;
   double kd;

   /* motor ids */
   int left_id;
   int right_id;

   /* serial devices we are interacting with the 8010 on */
   SerialPort * serial_l;
   SerialPort * serial_r; 

   /* input ros node */
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_in;

   /* periodic transmission values */
   double last_lwh, last_rwh;

   // https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/Twist.msg
   // https://en.wikipedia.org/wiki/Differential_wheeled_robot
   // https://techshare.co.jp/faq/wp-content/uploads/2023/12/GO-M8010-6_Motor_User_Manual_V1.0.pdf
   void collect_cmd_vel(const geometry_msgs::msg::Twist & msg) {

      double w = msg.angular.z;
      double V = msg.linear.x;

      double rwh = (V + (w * b2)) / r;
      double lwh = (V - (w * b2)) / r;

      write_values(lwh,rwh);
      last_lwh = lwh;
      last_rwh = rwh;
   }

   /* taken from:
    * https://github.com/unitreerobotics/unitree_actuator_sdk
    */
   void write_values(double lwh, double rwh) {
      MotorCmd cmd_l, cmd_r;
      MotorData data_l, data_r;

      double gearRatio = queryGearRatio(MotorType::GO_M8010_6);

      cmd_l.motorType = MotorType::GO_M8010_6;
      cmd_r.motorType = MotorType::GO_M8010_6;
      data_l.motorType = MotorType::GO_M8010_6;
      data_r.motorType = MotorType::GO_M8010_6;
      cmd_l.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
      cmd_r.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
      cmd_l.id = left_id;
      cmd_r.id = right_id;
      cmd_l.kp = cmd_r.kp = 0.0; /* we aren't using position right now */
      cmd_l.kd = cmd_r.kd = kd;
      cmd_l.q = cmd_r.q = 0.0;
      cmd_l.dq = lwh * gearRatio;
      cmd_r.dq = rwh * gearRatio;
      cmd_l.tau = cmd_r.tau = 0.0;

      try {
         serial_l->sendRecv(&cmd_l,&data_l);
         serial_r->sendRecv(&cmd_r,&data_r);
      } catch (IOException &) {
         perror("error");
      }
   }
};

int main(int argc, char ** argv) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<Go8010Driver>());
   rclcpp::shutdown();
   return 0;
}
