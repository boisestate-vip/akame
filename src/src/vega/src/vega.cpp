
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "tf2/LinearMath/Quaternion.h"

#include "visualization_msgs/msg/marker.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

#include "std_msgs/msg/empty.hpp"

#include "map.hpp"
#include "astar.hpp"

#include <stdio.h>
#include <cmath>
#include <ctime>

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

struct timespec tnow(void);
double timedist(struct timespec &t0, struct timespec &t1);

/* this is the actual ros node. The class is the node. */
class Vega : public rclcpp::Node {
public:

   Vega() : Node("vega") {

      /* topic to publish the produced map on */
      this->declare_parameter("map_in","/map");

      /* topic to listen for source-of-truth odometry on */
      this->declare_parameter("pos_in","/demo/odom");
      /* topic to listen for source-of-truth odometry on */
      this->declare_parameter("goal_in","/goal_pose");

      /* topic to listen on for a reset input */
      this->declare_parameter("reset_in","/vega/reset");

      /* the topic to publish the path on */
      this->declare_parameter("path_out","/path");
      /* interval in seconds to publish the map in */
      this->declare_parameter("path_publish_interval",0.1);

      /* the topic to publish the visualization on */
      this->declare_parameter("visual_out","/path_visual");
      /* the topic to publish the map we are using on */
      this->declare_parameter("map_out","/vega/map");
      /* whether to publish the visualization message */
      this->declare_parameter("publish_visual",true);

      /* parameters for the map */
      this->declare_parameter("map_obstacle_threshold",0.3); 
      /* parameters for the map */
      this->declare_parameter("map_resolution",0.1); /* meters */
      this->declare_parameter("map_hit_weight",80); 
      this->declare_parameter("map_miss_weight",10);
      this->declare_parameter("map_start_weight",50);

      this->map = GridMap(
         this->get_parameter("map_resolution").as_double(),
         this->get_parameter("map_hit_weight").as_int(),
         this->get_parameter("map_miss_weight").as_int(),
         this->get_parameter("map_start_weight").as_int(),
         10.0,10.0, // 10 meters by 10 meters seems conservative :/
         this->get_parameter("map_obstacle_threshold").as_double()
      );

      this->declare_parameter("repath_range",1.0);
      this->declare_parameter("full_repath_interval",3.0);

      /* now instantiate our subscriptions to our information sources */
      pos_in = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("pos_in").as_string(), 10,
            std::bind(&Vega::collect_pos, this, _1));

      goal_in = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            this->get_parameter("goal_in").as_string(), 10,
            std::bind(&Vega::collect_goal, this, _1));

      map_in = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            this->get_parameter("map_in").as_string(), 10,
            std::bind(&Vega::collect_map, this, _1));

      reset_in = this->create_subscription<std_msgs::msg::Empty>(
            this->get_parameter("reset_in").as_string(), 10,
            std::bind(&Vega::collect_reset, this, _1));

      /* setup the topic we will publish the computed map on */
      path_out = this->create_publisher<nav_msgs::msg::Path>(
            this->get_parameter("path_out").as_string(), 10);
      visual_out = this->create_publisher<visualization_msgs::msg::Marker>(
            this->get_parameter("visual_out").as_string(), 10);
      map_out = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            this->get_parameter("map_out").as_string(), 10);
      path_callback = this->create_wall_timer(
            std::chrono::milliseconds((long)(1000.0 *
                  this->get_parameter("path_publish_interval").as_double())),
            std::bind(&Vega::publish_path, this));

      have_goal = 0;
      repath_interval = this->get_parameter("full_repath_interval").as_double();
      repath_dist = this->get_parameter("repath_range").as_double();

      path_timeout = this->get_parameter("path_publish_interval").as_double();
   }

private:

   /* === node variables === */

   /* the current seen position of the robot */
   std::string ref_frame = "map";
   pose_2d last_pos;

   /* the goal we are trying to get to */
   point_2d goal;
   int have_goal;

   /* the map we are working off of */
   GridMap map;

   /* the pathfinding algorithm */
   AStar path;

   /* keeping track of when to repath */
   struct timespec last_repath;
   double repath_interval;

   /* saving the repath distance */
   double repath_dist;
   /* saving the publish interval */
   double path_timeout;

   /* the topic we are publishing the map on and the timer that
    * triggers it to publish at the given interval.            */
   rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_in;

   /* the topic we recieve odometry messages on */
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_in;

   /* the topic we recieve our reset commands on. */
   rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_in;

   /* the topic to recieve the goal position on. */
   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_in;

   /* the topic the path is published on */
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_out;
   /* the topic the visualization is published on */
   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visual_out;
   rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_out;
   /* path publish timer */
   rclcpp::TimerBase::SharedPtr path_callback;


   /* === node callback handlers === */

   // https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg
   void collect_pos(const nav_msgs::msg::Odometry & msg) {
      ref_frame = msg.header.frame_id;
      last_pos = ros2_pose_to_pose_2d(msg.pose.pose);
   }

   // https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/PoseStamped.msg
   void collect_goal(const geometry_msgs::msg::PoseStamped & p) {
      have_goal = 1;
      goal = point_2d{p.pose.position.x,p.pose.position.y};
      path.clear();
   }

   // https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/OccupancyGrid.msg
   void collect_map(const nav_msgs::msg::OccupancyGrid & msg) {
      if (map.from_msg(msg))
         path.clear();
      if (this->get_parameter("publish_visual").as_bool()) {
         nav_msgs::msg::OccupancyGrid msg;
         map.to_msg(msg);
         msg.header.frame_id = ref_frame;
         msg.header.stamp = this->get_clock()->now();
         map_out->publish(msg);
      }
   }

   // https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/Empty.msg
   void collect_reset(const std_msgs::msg::Empty & msg) {
      (void)msg;
      map.clear();
      path.clear();
   }

   // https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Path.msg
   void publish_path() {
      if ( ! have_goal) return;
      nav_msgs::msg::Path msg;

      /* you will note that we are treating the goal as the start.
       * This produces better results for repathing.              */

      struct timespec curr_time = tnow();
      if (timedist(last_repath,curr_time) > repath_interval || ! path.has_path()) {
         last_repath = tnow();
         path.route(goal.x,goal.y,last_pos.x,last_pos.y,path_timeout,&map);
      }
      else {
         path.reroute(repath_dist,last_pos.x,last_pos.y,path_timeout,&map);
      }

      path.to_msg(msg,&map);
      msg.header.frame_id = ref_frame;
      msg.header.stamp = this->get_clock()->now();

      path_out->publish(msg);

      /* a bit lazy to call this every time, but there is a way
       * to set up an interface to change this live, so I will
       * say I am preparing for that :)                        */
      if (this->get_parameter("publish_visual").as_bool()) {
         visualization_msgs::msg::Marker marker;
         path.to_vis(marker,&map);
         marker.header.frame_id = ref_frame;
         marker.header.stamp = this->get_clock()->now();

         visual_out->publish(marker);
      }
   }

};

int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<Vega>());
   rclcpp::shutdown();
   return 0;
}
