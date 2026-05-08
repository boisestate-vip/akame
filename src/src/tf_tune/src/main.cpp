
#include <memory>
#include <functional>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <string>
#include <chrono>
#include <termios.h>
#include <unistd.h>

#define TO_RAD(x) (x*(180.0 / M_PI))
#define TO_DEG(x) (x*(M_PI / 180.0))

class TFTune : public rclcpp::Node
{
public:
  TFTune() : Node("tf_tune") {

    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    printf("this is a program for tuning transforms.\n"
           "it was written by qeftser after he got upset\n"
           "at the thought of tuning the transforms by hand.\n"
           "I hope you enjoy it and find it useful.\n");

    printf("\nenter parent -> child frames..\n");

    char buf[128];

    fprintf(stderr,"enter parent frame: ");
    fgets(buf,127,stdin);
    buf[strlen(buf)-1] = '\0';
    printf("got parent frame as: %s\n",buf);

    parent = std::string(buf);

    fprintf(stderr,"enter child frame: ");
    fgets(buf,127,stdin);
    buf[strlen(buf)-1] = '\0';
    printf("got child frame as: %s\n",buf);

    child = std::string(buf);

    tf_callback = this->create_wall_timer(
       std::chrono::milliseconds(100),
       std::bind(&TFTune::make_transforms, this));

    bottom = std::thread(&TFTune::input,this);
  }

private:

   /* publisher */
   std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
   /* callback timer */
   rclcpp::TimerBase::SharedPtr tf_callback;

   /* reference frame to publish. Goes parent -> child */
   std::string parent, child;

   /* current values of the program */
   double x = 0.0, y = 0.0, z = 0.0;
   double roll = 0.0, pitch = 0.0, yaw = 0.0;

   /* thread that the command accepter runs in */
   std::thread bottom;

   void input() {

      struct termios orig;
      struct termios t;

      /* modify the terminal to be in raw input mode */
      int stdio_fd = fileno(stdin);
      int res0 = tcgetattr(stdio_fd,&t);
      orig = t;
      if (res0 == -1) { perror("tcgetattr"); exit(1); }
      cfmakeraw(&t);
      int res1 = tcsetattr(stdio_fd,TCSANOW,&t);
      if (res1 == -1) { perror("tcsetattr"); exit(1); }

      tcflush(stdio_fd,TCIOFLUSH);

      char cmd = '_';

      /* now we have a great raw terminal to use at our leisure */
      while (1) {
         printf("\033[H\033[2J");
         printf("usage:\r\n");
         printf("\t1/! - increase/decrease x by 0.01m\r\n");
         printf("\t2/@ - increase/decrease y by 0.01m\r\n");
         printf("\t3/# - increase/decrease z by 0.01m\r\n");
         printf("\tj/J - increase/decrease r by 0.01rad\r\n");
         printf("\tk/K - increase/decrease p by 0.01rad\r\n");
         printf("\tl/L - increase/decrease y by 0.01rad\r\n");
         printf("\tr - reset to 0\r\n");
         printf("\tq - quit everything\r\n");
         printf("\n\r\n");
         printf("X: %lf, Y: %lf, Z: %lf\r\n",x,y,z);
         printf("R: %lf, P: %lf, Y: %lf\r\n",roll,pitch,yaw);

         tf2::Quaternion q;
         q.setRPY(roll, pitch, yaw);
         printf("QX: %lf, QY: %lf, QZ: %lf, QW: %lf\r\n",q.x(),q.y(),q.z(),q.w());

         fprintf(stderr,"> %c",cmd);

         /* wait for next user input */
         cmd = getchar();
         switch (cmd) {
            case '1': x += 0.01; break;
            case '2': y += 0.01; break;
            case '3': z += 0.01; break;
            case 'j': roll += 0.01; break;
            case 'k': pitch += 0.01; break;
            case 'l': yaw += 0.01; break;
            case '!': x -= 0.01; break;
            case '@': y -= 0.01; break;
            case '#': z -= 0.01; break;
            case 'J': roll -= 0.01; break;
            case 'K': pitch -= 0.01; break;
            case 'L': yaw -= 0.01; break;
            case 'r': x=y=z=roll=pitch=yaw=0.0; break;
            case 'q': tcsetattr(stdio_fd,TCSANOW,&orig); exit(1); break;
            default: break;
         }
      }
   }

   void make_transforms(void) {
     geometry_msgs::msg::TransformStamped t;
 
     t.header.stamp = this->get_clock()->now();
     t.header.frame_id = parent;
     t.child_frame_id = child;
 
     t.transform.translation.x = x;
     t.transform.translation.y = y;
     t.transform.translation.z = z;
     tf2::Quaternion q;
     q.setRPY(roll, pitch, yaw);
     t.transform.rotation.x = q.x();
     t.transform.rotation.y = q.y();
     t.transform.rotation.z = q.z();
     t.transform.rotation.w = q.w();
 
     tf_static_broadcaster_->sendTransform(t);
   }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFTune>());
  rclcpp::shutdown();
  return 0;
}
