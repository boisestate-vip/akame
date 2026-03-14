
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

#include "map.hpp"
#include "points.hpp"
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

typedef struct point_2d {
   double x;
   double y;
} point_2d;

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
class Vega : public rclcpp::Node {
public:

   Vega() : Node("vega") {

      /* topic to publish the produced map on */
      this->declare_parameter("map_in","map");

      /* topic to listen for source-of-truth odometry on */
      this->declare_parameter("pos_in","/demo/odom");
      /* topic to listen for source-of-truth odometry on */
      this->declare_parameter("goal_in","/goal_pose");

      /* topic to listen on for a reset input */
      this->declare_parameter("reset_in","/vega/reset");

      /* the topic to publish the path on */
      this->declare_parameter("path_out","path");
      /* interval in seconds to publish the map in */
      this->declare_parameter("path_publish_interval",0.1);

      /* parameters for the map */
      this->declare_parameter("map_resolution",0.1); /* meters */
      this->declare_parameter("map_obstacle_threshold",40); 
      this->declare_parameter("map_circle_radius",0.3);

      this->declare_parameter("repath_range",1.0);
      this->declare_parameter("full_repath_interval",3.0);

      /* now instantiate our subscriptions to our information sources */
      pos_in = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("pos_in").as_string(), 10,
            std::bind(&Vega::collect_pos, this, _1));

      goal_in = this->create_subscription<nav_msgs::msg::PoseStamped>(
            this->get_parameter("goal_in").as_string(), 10,
            std::bind(&Vega::collect_goal, this, _1));

      map_in = this->create_subscription<sensor_msgs::msg::OccupancyGrid>(
            this->get_parameter("map_in").as_string(), 10,
            std::bind(&Vega::collect_map, this, _1));

      reset_in = this->create_subscription<std_msgs::msg::Empty>(
            this->get_parameter("reset_in").as_string(), 10,
            std::bind(&Vega::collect_reset, this, _1));

      /* setup the topic we will publish the computed map on */
      path_out = this->create_publisher<nav_msgs::msg::Path>(
            this->get_parameter("path_out").as_string(), 10);
      path_callback = this->create_wall_timer(
            std::chrono::milliseconds((long)(1000.0 *
                  this->get_parameter("path_publish_interval").as_double())),
            std::bind(&Vega::publish_path, this));


   }

private:

   /* === node variables === */

   /* the current seen position of the robot */
   pose_2d last_pos;

   /* the goal we are trying to get to */
   point_2d goal;

   /* the topic we are publishing the map on and the timer that
    * triggers it to publish at the given interval.            */
   rclcpp::Subscriber<nav_msgs::msg::OccupancyGrid>::SharedPtr map_in;

   /* the topic we recieve odometry messages on */
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_in;

   /* the topic we recieve lidar input on */
   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_in;

   /* the topic we recieve our reset commands on. */
   rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_in;

   /* the topic the path is published on */
   rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_in;
   /* path publish timer */
   rclcpp::TimerBase::SharedPtr path_callback;


   /* === node callback handlers === */

   // https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg
   void collect_pos(const nav_msgs::msg::Odometry & msg) {
      last_pos = ros2_pose_to_pose_2d(msg.pose.pose);
   }

   // https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/PoseStamped.msg
   void collect_goal(consst geometry_msgs::msg::PoseStamped & p) {
      goal = point_2d{p.pose.position.x,p.pose.position.y};
   }

   // https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg
   void collect_map(const nav_msgs::msg::OccupancyGrid & msg) {
   }

   // https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg
   void collect_points(const sensor_msgs::msg::PointCloud2 & msg) {
      /* sort of lifted from my pino implimentation */
      static int fail_count = 0;

      /* convert the point cloud into the
       * frame of reference for base_link */
      try {
         /* this guy is a bit better than the laser scan in that we can directly
          * transform it into the reference frame we want without having to manually
          * apply the transforms.                                                   */
         sensor_msgs::msg::PointCloud2 cloud_out;
         tf_buffer->transform<sensor_msgs::msg::PointCloud2>(msg,cloud_out,"odom",
               tf2::Duration(std::chrono::milliseconds(500)));

         // we use last pos here because we don't have the transform
         Points data = cloud_to_points(cloud_out, last_pos); 

         map.add_points(data);
         fail_count = 0;

      }
      catch(const tf2::TransformException & ex) {
         RCLCPP_WARN(this->get_logger(),"transformation of point cloud failed for the %dth time: %s to %s",fail_count++,msg.header.frame_id.c_str(),"map");
         return;
      }
   }

   // https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/Empty.msg
   void collect_reset(const std_msgs::msg::Empty & msg) {
      (void)msg;
      map.clear();
   }

   // https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/OccupancyGrid.msg
   void publish_map() {
      nav_msgs::msg::OccupancyGrid msg;
      map.to_msg(msg);

      msg.info.map_load_time = this->get_clock()->now();
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "map";

      map_out->publish(msg);
   }

   // https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/TransformStamped.msg
   void broadcast_map_frame() {
      geometry_msgs::msg::TransformStamped msg;
      msg.header.frame_id = "map";
      msg.child_frame_id = "odom";

      /* because we are not doing any kind of slam or
       * correction at this stage, we are basically
       * assuming there is no difference between the map
       * and the odom frames of reference.               */
      pose_2d diff = { 0, 0, 0 };

      /* map back into ros's representation */
      msg.transform.translation.x = diff.x;
      msg.transform.translation.y = diff.y;
      msg.transform.translation.z = 0.0;
      tf2::Quaternion q; q.setRPY(0.0,0.0,diff.t);
      msg.transform.rotation.x = q.getX();
      msg.transform.rotation.y = q.getY();
      msg.transform.rotation.z = q.getZ();
      msg.transform.rotation.w = q.getW();

      msg.header.stamp = this->get_clock()->now();
      map_frame->sendTransform(msg);
   }

};

int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<SimpleMap>());
   rclcpp::shutdown();
   return 0;
}
