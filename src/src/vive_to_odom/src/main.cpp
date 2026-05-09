
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "builtin_interfaces/msg/time.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Transform.h"

#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "std_msgs/msg/empty.hpp"

#include "stdio.h"

#include <cmath>

using std::placeholders::_1;

/* this is the actual ros node. The class is the node. */
class ViveToOdom : public rclcpp::Node {
public:

   ViveToOdom() : Node("vive_to_odom") {

      /* interval in seconds to publish the transform in */
      this->declare_parameter("frame_publish_interval",0.005);

      /* topic to listen for source-of-truth odometry on */
      this->declare_parameter("pos_in","/demo/odom");

      /* topic to listen for source-of-truth odometry on */
      this->declare_parameter("pos_out","/odom");

      /* now instantiate our subscriptions to our information sources */
      pos_in = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("pos_in").as_string(), 10,
            std::bind(&ViveToOdom::collect_pos, this, _1));

      /* publisher intantiation */
      pos_out = this->create_publisher<nav_msgs::msg::Odometry>(
            this->get_parameter("pos_out").as_string(), 10);

      /* this is the setup we will be using for getting and publishing transform data */
      tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());

      tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

      odom_frame = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
   }

private:

   /* === node variables === */

   /* publish the odom transform frame. Also the timer that
    * specifies the publish interval.
    *
    * This is done according to the following ros standard:
    * https://www.ros.org/reps/rep-0105.html
    */
   std::unique_ptr<tf2_ros::TransformBroadcaster> odom_frame;
   rclcpp::TimerBase::SharedPtr odom_frame_callback;

   /* the topic we recieve odometry messages on */
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_in;

   /* the topic we publish on */
   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pos_out;

   /* Used to transform incoming scan data into the base_link
    * reference frame for the robot.                                */
   std::unique_ptr<tf2_ros::TransformListener> tf_listener;
   /* Performs all nessesary conversions between different 
    * reference frames.                                             */
   std::unique_ptr<tf2_ros::Buffer> tf_buffer;

   /* === node callback handlers === */

   // https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg
   void collect_pos(const nav_msgs::msg::Odometry & msg) {
      static int fail_count = 0;

      geometry_msgs::msg::TransformStamped t;
      std::string frame_id = msg.header.frame_id;

      try {
         t = tf_buffer->lookupTransform(
               "base_link",frame_id,tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
         RCLCPP_WARN(
            this->get_logger(), "[%d] Could not transform %s to base_link: %s",
            fail_count,frame_id.c_str(), ex.what());
         fail_count += 1;
         return;
      }

      auto static_t = tf2::Transform(
         tf2::Quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
         ),
         tf2::Vector3(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
         )
      );
      auto odom_transform = tf2::Transform(
         tf2::Quaternion(
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
         ),
         tf2::Vector3(
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z
         )
      );

      auto transformed = static_t * odom_transform;

      geometry_msgs::msg::TransformStamped odom_to_base_link;
      odom_to_base_link.header = msg.header;
      odom_to_base_link.header.frame_id = "odom";
      odom_to_base_link.child_frame_id = "base_link";

      odom_to_base_link.transform.translation.x = transformed.getOrigin().x();
      odom_to_base_link.transform.translation.y = transformed.getOrigin().y();
      odom_to_base_link.transform.translation.z = transformed.getOrigin().z();

      odom_to_base_link.transform.rotation.x = transformed.getRotation().x();
      odom_to_base_link.transform.rotation.y = transformed.getRotation().y();
      odom_to_base_link.transform.rotation.z = transformed.getRotation().z();
      odom_to_base_link.transform.rotation.w = transformed.getRotation().w();

      odom_frame->sendTransform(odom_to_base_link);

      nav_msgs::msg::Odometry odom_base_link;
      odom_base_link.header = msg.header;
      odom_base_link.header.frame_id = "odom";

      odom_base_link.pose.pose.position.x = transformed.getOrigin().x();
      odom_base_link.pose.pose.position.y = transformed.getOrigin().y();
      odom_base_link.pose.pose.position.z = transformed.getOrigin().z();

      odom_base_link.pose.pose.orientation.x = transformed.getRotation().x();
      odom_base_link.pose.pose.orientation.y = transformed.getRotation().y();
      odom_base_link.pose.pose.orientation.z = transformed.getRotation().z();
      odom_base_link.pose.pose.orientation.w = transformed.getRotation().w();

      pos_out->publish(odom_base_link);
   }

};

int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<ViveToOdom>());
   rclcpp::shutdown();
   return 0;
}
