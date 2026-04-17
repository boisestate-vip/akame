
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

#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "std_msgs/msg/empty.hpp"

#include "band.hpp"
#include "stdio.h"

#include <cmath>

/* the first two here are checking for straighforward illigal
 * floating values. The last guy takes advantage of a property
 * that a nan or inf will never compare equal to itself.
 *
 * The x != x check I think is one of the only good cross
 * platform ways to determine floating point nan/inf, though
 * it is very odd to look at...                              */
#define BAD_VAL(x) (std::isnan(x) || std::isinf(x) || x != x)

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

/* this is the actual ros node. The class is the node. */
class Altair : public rclcpp::Node {
public:

   Altair() : Node("altair") {

      /* topic to publish the produced map on */
      this->declare_parameter("map_in","map");

      /* topic to listen for source-of-truth odometry on */
      this->declare_parameter("pos_in","/demo/odom");
      /* topic to recieve the goal on */
      this->declare_parameter("goal_in","/goal");
      /* topic to accept the path to smooth */
      this->declare_parameter("path_in","/path");

      /* topic to listen on for a reset input */
      this->declare_parameter("reset_in","/simple_map/reset");

      /* topic to publish the smoothed path on */
      this->declare_parameter("path_out","/path_smooth");

      /* interval in seconds to publish the path in */
      this->declare_parameter("path_publish_interval",0.1);

      /* map specific parameters */
      this->declare_parameter("internal_multiplier",1.0);
      this->declare_parameter("external_multiplier",1.0);
      this->declare_parameter("constraint_multiplier",1.0);
      this->declare_parameter("distance_cutoff",1.0);
      this->declare_parameter("map_height",40);
      this->declare_parameter("bin_width",0.3);
      this->declare_parameter("max_length",3.0);
      this->declare_parameter("hit_distance",0.2);

      /* now instantiate our subscriptions to our information sources */
      pos_in = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("pos_in").as_string(), 10,
            std::bind(&Altair::collect_pos, this, _1));

      goal_in = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            this->get_parameter("goal_in").as_string(), 10,
            std::bind(&Altair::collect_goal, this, _1));

      path_in = this->create_subscription<nav_msgs::msg::Path>(
            this->get_parameter("path_in").as_string(), 10,
            std::bind(&Altair::collect_path, this, _1));

      map_in = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            this->get_parameter("map_in").as_string(), 10,
            std::bind(&Altair::collect_map, this, _1));

      reset_in = this->create_subscription<std_msgs::msg::Empty>(
            this->get_parameter("reset_in").as_string(), 10,
            std::bind(&Altair::collect_reset, this, _1));

      /* setup the topic we will publish the computed path on */
      path_out = this->create_publisher<nav_msgs::msg::Path>(
            this->get_parameter("path_out").as_string(), 10);
      path_callback = this->create_wall_timer(
         std::chrono::milliseconds(((long)(1000.0 * 
                  this->get_parameter("path_publish_interval").as_double()))),
         std::bind(&Altair::publish_path, this));

      band = Band(
         this->get_parameter("internal_multiplier").as_double(),
         this->get_parameter("external_multiplier").as_double(),
         this->get_parameter("constraint_multiplier").as_double(),
         this->get_parameter("distance_cutoff").as_double(),
         (uint8_t)this->get_parameter("map_height").as_int(),
         this->get_parameter("bin_width").as_double(),
         this->get_parameter("max_length").as_double(),
         this->get_parameter("hit_distance").as_double()
      );
   }

private:

   /* === node variables === */

   /* the current seen position of the robot */
   pose_2d pos;

   /* the current goal */
   pose_2d goal;

   /* holds the elastic band */
   Band band;

   /* the topic we recieve the map on */
   rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_in;

   /* the topic we recieve odometry messages on */
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_in;

   /* the topic we recieve lidar input on */
   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_in;

   /* the topic we recieve the path to smooth on */
   rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_in;

   /* the topic we recieve our reset commands on. */
   rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_in;

   /* the topic to publish the smoothed path from */
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_out;
   rclcpp::TimerBase::SharedPtr path_callback;

   /* === node callback handlers === */

   // https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg
   void collect_pos(const nav_msgs::msg::Odometry & msg) {
      pos = ros2_pose_to_pose_2d(msg.pose.pose);
   }

   // https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/PoseStamped.msg
   void collect_goal(const geometry_msgs::msg::PoseStamped & msg) {
      goal = ros2_pose_to_pose_2d(msg.pose);
      band.clear();
   }

   // https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Path.msg
   void collect_path(const nav_msgs::msg::Path & msg) {
      if ( ! band.has_path())
         band.load_path(msg);
   }

   // https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/Empty.msg
   void collect_reset(const std_msgs::msg::Empty & msg) {
      (void)msg;
      band.clear();
   }

   void collect_map(const nav_msgs::msg::OccupancyGrid & msg) {
      band.load_map(msg);
   }

   // https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Path.msg
   void publish_path() {

      if ( ! band.has_path())
         return;

      band.step(pos.x,pos.y,10);

      nav_msgs::msg::Path msg;


      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "map";
      band.to_msg(msg);

      path_out->publish(msg);
   }

};

int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<Altair>());
   rclcpp::shutdown();
   return 0;
}
