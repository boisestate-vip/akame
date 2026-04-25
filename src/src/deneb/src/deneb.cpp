
#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* this first bit is some ported math code from kurome.
 * It took me quite a while to get the right equations
 * for mapping from the 3d + quaternion space of ros
 * into the simpler 2d + theta space we work in. Anyway,
 * the functions are duplicated here with no commentary. */
typedef struct pose_2d {
   double x;
   double y;
   double t;
} pose_2d;

double quaternion_to_z_rotation(const geometry_msgs::msg::Quaternion & q) {
   double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
   double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
   return std::atan2(siny_cosp,cosy_cosp);
}

pose_2d ros2_pose_to_pose_2d(const geometry_msgs::msg::Pose & p) {
   return {p.position.x,p.position.y,quaternion_to_z_rotation(p.orientation)};
}

geometry_msgs::msg::Pose pose_2d_to_ros2_pose(const pose_2d & p) {
   tf2::Quaternion q; q.setRPY(0.0,0.0,p.t);

   geometry_msgs::msg::Pose ret;
   ret.position.x = p.x;
   ret.position.y = p.y;
   ret.position.z = 0.0;

   ret.orientation.x = q.getX();
   ret.orientation.y = q.getY();
   ret.orientation.z = q.getZ();
   ret.orientation.w = q.getW();

   return ret;
}

class Deneb : public rclcpp::Node {
public:
   Deneb(): Node("deneb") {

    /* topic to publish the path on */
    this->declare_parameter("path_in","/path_smooth");

    /* topic to publish the odometry on */
    this->declare_parameter("pos_in","/demo/odom");

    /* topic to publish the velocity command on */
    this->declare_parameter("vel_out","/demo/cmd_vel");

    /* visualization stuff */
    this->declare_parameter("publish_visual",false);
    this->declare_parameter("visual_out","/follower_visual");

    /* rate to publish commands at */
    this->declare_parameter("vel_publish_interval",0.1);

    subscriber = this->create_subscription<nav_msgs::msg::Path>(
                 this->get_parameter("path_in").as_string(),10,
                 std::bind(&Deneb::path_callback, this, _1));

    pos_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                 this->get_parameter("pos_in").as_string(),10,
                 std::bind(&Deneb::pos_callback, this, _1));

    cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(
          this->get_parameter("vel_out").as_string(),10);

    visual = this->create_publisher<visualization_msgs::msg::MarkerArray>(
          this->get_parameter("visual_out").as_string(), 10);

    timer = this->create_wall_timer(
          std::chrono::milliseconds(((long)(1000.0 *
                this->get_parameter("vel_publish_interval").as_double()))),
          std::bind(&Deneb::timer_callback, this));
  }

private:

  /* topic we recieve the path on */
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscriber;

  /* topic we recieve the robot position on */
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_sub;

  /* topic we publish velocity on */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;

  /* topic we publish our visualization on */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visual;

  rclcpp::TimerBase::SharedPtr timer;

  nav_msgs::msg::Path path;
  pose_2d current_pos;

  void path_callback(const nav_msgs::msg::Path & msg) {
     path = msg;
  }

  void pos_callback(const nav_msgs::msg::Odometry & msg) {
     current_pos = ros2_pose_to_pose_2d(msg.pose.pose);
  }

  void timer_callback() {
     /*
     geometry_msgs::msg::Twist msg;
     msg.header = path.header;
     msg.header.stamp = this->get_clock()->now();
     */

     geometry_msgs::msg::Twist twist;

     double linear_vel = 0;
     double angular_vel = 0;

     /* now we want to actually compute the
      * movement to take in order to follow
      * the path. We assume that we only get
      * a new path when the goal changes, so
      * we go ahead directly modify the path
      * we do get.                           */

     if (path.poses.size() == 0) {
        goto publish;
     }
     else {

        double dist;
        pose_2d next_2d;

        do {
           geometry_msgs::msg::PoseStamped next = path.poses.back();
           next_2d = ros2_pose_to_pose_2d(next.pose);

           /* compute the distance between robot and next path node 
            * Note that this is literally just an application of the
            * pythagorean theorem */
           dist = sqrt( (next_2d.x - current_pos.x) *
                        (next_2d.x - current_pos.x) +
                        (next_2d.y - current_pos.y) *
                        (next_2d.y - current_pos.y)  );

           /* if we are within 0.25 (meters?) of the goal, consider
            * the node reached and go to the next one.             */
           if (dist < 0.25) {

              path.poses.pop_back();

              /* check again to see if we have reached the end */
              if (path.poses.size() == 0)
                 goto publish;
           }
           else break;

        } while (1);

        /* perform the translation before the rotation */
        double rel_x = next_2d.x - current_pos.x;
        double rel_y = next_2d.y - current_pos.y;

        /* note that if we transform the robot in this way
         * we get the equations:
         *
         *  rel_x = current_pos.pos.x - current_pos.pos.x
         *  rel_y = current_pos.pos.y - current_pos.pos.y
         *
         * This clearly moves the robot to 0,0  */

        /* transform the goal position into the robot's
         * frame of reference using an inverse rotation
         * matrix.                                      */
        double t_x = (cos(current_pos.t) * rel_x) +
                     (sin(current_pos.t) * rel_y);
        double t_y = (cos(current_pos.t) * rel_y) -
                     (sin(current_pos.t) * rel_x);

        /* note that we can still use the dist value, as the distance does not
         * change if we perform the same transformation on both points       */

        /* went here to get the equations for solving this. 
         * A is the angle 90 degrees because we know this is a right triangle
         * a is the value of dist that we have already computed
         * B is the angle we want
         * b is the value of t_y (distance in the y direction of the robot from the goal)
         *
         * We know that sin(90) is 1 so we can ignore it in the math...
         *
         * final equation is t_y * (sin(90) / dist) == (t_y / dist)
         * We then take the inverse sin of this to get the angle we want
         *
         * https://www.mathsisfun.com/algebra/trig-solving-sas-triangles.html */

        double angular_error = asin(t_y / dist);

        /* if this value is less than some arbitrary amount, drive forward,
         * otherwise rotate in the opposite direction.                     */
        if (fabs(angular_error) < 0.25)
           linear_vel = 0.1;
        else angular_vel = (angular_error < 0 ? -1 : 1) * 0.15;
     }

publish:

     twist.linear.x = linear_vel;
     twist.linear.y = 0;
     twist.linear.z = 0;
     twist.angular.z = 0;
     twist.angular.y = 0;
     twist.angular.z = angular_vel;

     /*
     msg.twist = twist;
     cmd_pub->publish(msg);
     */
     cmd_pub->publish(twist);

     if (this->get_parameter("publish_visual").as_bool()) {
        /* fancy visualization of the path remaining */
        uint64_t marker_id = 0;
        visualization_msgs::msg::MarkerArray markers;
        for (geometry_msgs::msg::PoseStamped & msg : path.poses) {

           visualization_msgs::msg::Marker marker;

           marker.header = msg.header;
           marker.pose = msg.pose;
           marker.action = 0;
           marker.scale.x = 0.25;
           marker.scale.y = 0.25;
           marker.scale.z = 0.25;
           marker.type = 2;
           marker.id = ++marker_id;
           marker.ns = "path_markers";
           marker.lifetime.nanosec = 7e8;
           marker.lifetime.sec = 0;
           marker.color.r = 1.0;
           marker.color.g = 0.0;
           marker.color.b = 0.0;
           marker.color.a = 0.5;

           markers.markers.push_back(marker);
        }

        visual->publish(markers);
     }
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Deneb>());
  rclcpp::shutdown();
  return 0;
}
