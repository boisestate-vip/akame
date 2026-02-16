
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

using std::placeholders::_1;

/* best effort, https://www.man7.org/linux/man-pages/man3/termios.3.html */
static speed_t get_baud_const(int baud) {
   static speed_t vals[] = {
      B0,
      B50,
      B75,
      B110,
      B134,
      B150,
      B200,
      B300,
      B600,
      B1200,
      B1800,
      B2400,
      B4800,
      B9600,
      B19200,
      B38400,
      B57600,
      B115200,
      B230400,
      B460800,
      B500000,
      B576000,
      B921600,
      B1000000,
      B1152000,
      B1500000,
      B2000000,
   };
   static int baud_nums[] = {
      0,
      50,
      75,
      110,
      134,
      150,
      200,
      300,
      600,
      1200,
      1800,
      2400,
      4800,
      9600,
      19200,
      38400,
      57600,
      115200,
      230400,
      460800,
      500000,
      576000,
      921600,
      1000000,
      1152000,
      1500000,
      2000000,
   };

   /* position ourselves in between the cloest values */
   int idx = 0;
   while (baud > baud_nums[idx] && idx < 26)
      idx += 1;

   /* crappy baud given */
   if (idx == 26 || idx == 0)
      return vals[idx];

   /* exact match */
   if (baud == baud_nums[idx])
      return vals[idx];

   int dist_low = std::abs(baud_nums[idx-1] - baud);
   int dist_high = std::abs(baud_nums[idx] - baud);

   /* closest value */
   if (dist_low > dist_high)
      return vals[idx];
   return vals[idx-1];
}

/* try and connect and set the given baud rate for the waveshare */
static int try_connect(const char * device, int baud) {

   int fd = open(device,O_WRONLY|O_NONBLOCK);
   if (fd < 0) {
      perror("open");
      return -1;
   }

   struct termios t;
   int res0 = tcgetattr(fd,&t);
   if (res0 < 0) {
      perror("tcgetattr");
      return -1;
   }

   int res1 = cfsetispeed(&t,get_baud_const(baud));
   if (res1 < 0) {
      perror("cfsetispeed");
      return -1;
   }

   return fd;
}

class WaveshareDriver : public rclcpp::Node {
public:

   WaveshareDriver() : Node("waveshare_driver") {

      /* incoming command inputs */
      this->declare_parameter("cmd_in","cmd_vel");

      /* the device to connect to for interacting with the waveshare
       * PLEASE set this to a serial input, otherwise you could get
       * weird issues.                                             */
      this->declare_parameter("waveshare_device","/dev/serial/by-id/***");

      /* the baud rate to communicate with the waveshare robot on */
      // https://www.waveshare.com/wiki/WAVE_ROVER#Issue_JSON_Commands_Using_GPIO_or_USB_Serial_Port
      this->declare_parameter("waveshare_baud",115200);

      /* the radius of the wheels for the differential drive calculation (meters) */
      this->declare_parameter("wheel_radius",1.0);

      /* the width of the vehicle for the differential drive calculation (meters) */
      this->declare_parameter("vehicle_width",1.0);

      /* setup variables */
      cmd_in = this->create_subscription<geometry_msgs::msg::Twist>(
            this->get_parameter("cmd_in").as_string(), 10,
            std::bind(&WaveshareDriver::collect_cmd_vel, this, _1));

      /* save the diff-drive values */
      b2 = this->get_parameter("wheel_radius").as_double() / 2.0;
      r = this->get_parameter("vehicle_width").as_double();


      fd = try_connect(this->get_parameter("waveshare_device").as_string().c_str(),
                       this->get_parameter("waveshare_baud").as_int());

      if (fd < 0) {
         RCLCPP_ERROR(this->get_logger(),"failed connecting to waveshare on device %s\n",
                     this->get_parameter("waveshare_device").as_string().c_str());
         exit(1);
      }
      else {
         RCLCPP_INFO(this->get_logger(),"connected to waveshare on device %s with baud %ld\n",
                     this->get_parameter("waveshare_device").as_string().c_str(),
                     this->get_parameter("waveshare_baud").as_int());
      }
   }

private:

   /* class variables */

   /* constants for the diff-drive computation */
   double b2;
   double r;

   /* descriptor we are interacting with the waveshare on */
   int fd;

   /* input ros node */
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_in;


   // https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/Twist.msg
   // https://en.wikipedia.org/wiki/Differential_wheeled_robot
   // https://www.waveshare.com/wiki/WAVE_ROVER#Chassis_Movement
   void collect_cmd_vel(const geometry_msgs::msg::Twist & msg) {

      double w = msg.angular.z;
      double V = msg.linear.x;

      double rwh = (V + (w * b2)) / r;
      double lwh = (V - (w * b2)) / r;

      char buf[256];
      memset(buf,0,sizeof(char)*256);
      snprintf(buf,255,"{\"T\":1,\"L\":%.10f,\"R\":%.10f}\n",lwh,rwh);

      int buflen = strlen(buf);

      ssize_t total = 0;
      do {
         ssize_t res = write(fd,buf+total,buflen-total);

         if (res == -1) {
            if (errno == EINTR)
               continue;
            RCLCPP_INFO(this->get_logger(),"failed writing to waveshare with error %s",strerror(errno));
            errno = 0; /* reset error */
            break;
         }

         total += res;

      } while (total < buflen);
   }

};

int main(int argc, char ** argv) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<WaveshareDriver>());
   rclcpp::shutdown();
   return 0;
}
