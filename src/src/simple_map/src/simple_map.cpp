
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
   printf("converting from %f %f ...\n",p.position.x,p.position.y);
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

/* this is the transform math I talked about on the 
 * board on friday. We bascially first drop the laser
 * scan messages into a 2d coordiate frame and then
 * apply our transforms to it.                       */
Points scan_to_points(const sensor_msgs::msg::LaserScan & scan, pose_2d & transform) {

   Points data = Points(transform.x,transform.y);

   printf("got transform as: %f %f %f\n",transform.x,transform.y,transform.t);

   /* it is worth precomputing these */
   double cost = std::cos(transform.t);
   double sint = std::sin(transform.t);

   double angle = scan.angle_min;
   for (size_t i = 0; i < scan.ranges.size(); ++i) {

      if (scan.ranges[i] < scan.range_min || scan.ranges[i] > scan.range_max)
         continue; /* skip anything outside of range */

      /* these values are relative to the sensor */
      double raw_x = std::cos(angle) * scan.ranges[i];
      double raw_y = std::sin(angle) * scan.ranges[i];

      /* now we convert into the 'real' coordinates */
      double x = raw_x * cost - raw_y * sint + transform.x;
      double y = raw_x * sint + raw_y * cost + transform.y;
      printf("transforming from %f %f to %f %f\n",raw_x,raw_y,x,y);

      /* insert the point into our collection */
      data.add_point(point2{x, y});

      angle += scan.angle_increment;
   }

   return data;
}

/* point cloud 2 is a bit of an interesting message type. It
 * basically declares fields and counts in a large array that
 * you are supposed to index into based on the format. It makes
 * sense when you are trying to be as general as possible, but
 * it is frankly a bit of a pain to work with...               */
Points cloud_to_points(const sensor_msgs::msg::PointCloud2 & cloud, pose_2d & transform) {

   Points data = Points(transform.x,transform.y);

   int32_t xoff = -1;
   int32_t xtype = 0; // we will do 0 for float, 1 for double
   int32_t yoff = -1;
   int32_t ytype = 0; // we will do 0 for float, 1 for double

   /* load in the values we want */
   for (const sensor_msgs::msg::PointField & field : cloud.fields) {
      if (field.name == "x") {
         if (field.datatype == 7) {
            xoff = field.offset;
            xtype = 0;
         }
         else if (field.datatype == 8) {
            xoff = field.offset;
            xtype = 1;
         }
      }
      if (field.name == "y") {
         if (field.datatype == 7) {
            yoff = field.offset;
            ytype = 0;
         }
         else if (field.datatype == 8) {
            yoff = field.offset;
            ytype = 1;
         }
      }
   }

   /* we will not do anything if we can't get the values we need */
   if (xoff == -1 || yoff == -1)
      return data;

   /* now actually grab the points */
   /* i am going to apoligize for this... it is the fastest way though... */

   switch (xtype|(ytype<<1)) {

      case 0: // float x, float y
         for (size_t i = 0; i < cloud.data.size(); i += cloud.point_step) {
            float x, y;
            memcpy(&x,&cloud.data[i+xoff],sizeof(float));
            memcpy(&y,&cloud.data[i+yoff],sizeof(float));
            data.add_point(point2{(double)x,(double)y});
         }
         break;
      case 1: // double x, float y
         for (size_t i = 0; i < cloud.data.size(); i += cloud.point_step) {
            double x;
            float y;
            memcpy(&x,&cloud.data[i+xoff],sizeof(double));
            memcpy(&y,&cloud.data[i+yoff],sizeof(float));
            data.add_point(point2{x,(double)y});
         }
         break;
      case 2: // float x, double y
         for (size_t i = 0; i < cloud.data.size(); i += cloud.point_step) {
            float x;
            double y;
            memcpy(&x,&cloud.data[i+xoff],sizeof(float));
            memcpy(&y,&cloud.data[i+yoff],sizeof(double));
            data.add_point(point2{(double)x,y});
         }
         break;
      case 3: // double x, double y
         for (size_t i = 0; i < cloud.data.size(); i += cloud.point_step) {
            double x, y;
            memcpy(&x,&cloud.data[i+xoff],sizeof(double));
            memcpy(&y,&cloud.data[i+yoff],sizeof(double));
            data.add_point(point2{x,y});
         }
         break;

   }

   return data;
}

/* this is the actual ros node. The class is the node. */
class SimpleMap : public rclcpp::Node {
public:

   SimpleMap() : Node("simple_map") {

      /* topic to publish the produced map on */
      this->declare_parameter("map_out","map");
      /* interval in seconds to publish the map in */
      this->declare_parameter("map_publish_interval",0.25);
      /* whether to publish the map frame of reference */
      this->declare_parameter("publish_map_frame",true);

      /* topic to listen for source-of-truth odometry on */
      this->declare_parameter("pos_in","/demo/odom");
      /* topic to listen for incoming point cloud data on */
      this->declare_parameter("points_in","/points");
      /* topic to listen for incoming lidar data on */
      this->declare_parameter("scan_in","/demo/scan");

      /* topic to listen on for a reset input */
      this->declare_parameter("reset_in","/simple_map/reset");

      /* parameters for the map */
      this->declare_parameter("map_resolution",0.1); /* meters */
      this->declare_parameter("map_hit_weight",40); 
      this->declare_parameter("map_miss_weight",10);
      this->declare_parameter("map_start_weight",50);

      /* now instantiate our subscriptions to our information sources */
      pos_in = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("pos_in").as_string(), 10,
            std::bind(&SimpleMap::collect_pos, this, _1));

      scan_in = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->get_parameter("scan_in").as_string(), 10,
            std::bind(&SimpleMap::collect_scan, this, _1));

      points_in = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("points_in").as_string(), 10,
            std::bind(&SimpleMap::collect_points, this, _1));

      reset_in = this->create_subscription<std_msgs::msg::Empty>(
            this->get_parameter("reset_in").as_string(), 10,
            std::bind(&SimpleMap::collect_reset, this, _1));

      /* setup the topic we will publish the computed map on */
      map_out = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            this->get_parameter("map_out").as_string(), 10);
      map_callback = this->create_wall_timer(
            std::chrono::milliseconds((long)(1000.0 *
                  this->get_parameter("map_publish_interval").as_double())),
            std::bind(&SimpleMap::publish_map, this));

      /* this is the setup we will be using for getting and publishing transform data */
      tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());

      tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

      if (this->get_parameter("publish_map_frame").as_bool()) {
         map_frame = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
         map_frame_callback = this->create_wall_timer(
               std::chrono::milliseconds(((long)(1000.0 *
                        this->get_parameter("map_publish_interval").as_double()))),
               std::bind(&SimpleMap::broadcast_map_frame, this));
      }

      /* now initialize the map */
      map = GridMap(
         this->get_parameter("map_resolution").as_double(),
         this->get_parameter("map_hit_weight").as_int(),
         this->get_parameter("map_miss_weight").as_int(),
         this->get_parameter("map_start_weight").as_int(),
         10.0,10.0 // 10 meters by 10 meters seems conservative :/
      );

      /* ensure that map frame is done */
      broadcast_map_frame();
   }

private:

   /* === node variables === */

   /* the map we are building and publishing */
   GridMap map;

   /* the current seen position of the robot */
   pose_2d last_pos;

   /* the topic we are publishing the map on and the timer that
    * triggers it to publish at the given interval.            */
   rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_out;
   rclcpp::TimerBase::SharedPtr map_callback;

   /* publish the map transform frame. Also the timer that
    * specifies the publish interval.
    *
    * This is done according to the following ros standard:
    * https://www.ros.org/reps/rep-0105.html
    */
   std::unique_ptr<tf2_ros::TransformBroadcaster> map_frame;
   rclcpp::TimerBase::SharedPtr map_frame_callback;

   /* the topic we recieve odometry messages on */
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_in;

   /* the topic we recieve lidar input on */
   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_in;

   /* the topic we recieve point cloud input on */
   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_in;

   /* the topic we recieve our reset commands on. */
   rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_in;

   /* Used to transform incoming scan data into the base_link
    * reference frame for the robot.                                */
   std::unique_ptr<tf2_ros::TransformListener> tf_listener;
   /* Performs all nessesary conversions between different 
    * reference frames.                                             */
   std::unique_ptr<tf2_ros::Buffer> tf_buffer;

   /* === node callback handlers === */

   // https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg
   void collect_pos(const nav_msgs::msg::Odometry & msg) {
      last_pos = ros2_pose_to_pose_2d(msg.pose.pose);
      RCLCPP_INFO(this->get_logger(),"got pose as : %f %f %f\n",last_pos.x,last_pos.y,last_pos.t);
   }

   // https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg
   void collect_scan(const sensor_msgs::msg::LaserScan & msg) {
      /* sort of lifted from my pino implimentation */
      static int fail_count = 0;

      /* convert the lidar data into the frame of
       * reference for base_link.               */
      try {
         /* this is us getting the transform needed to move from the frame of
          * reference of the map. We can then take this transform and apply it
          * to the points of the lidar to get the observation positions.      */
         geometry_msgs::msg::PoseStamped pose_in;
         geometry_msgs::msg::PoseStamped pose_out;
         pose_in.header = msg.header;
         tf_buffer->transform<geometry_msgs::msg::PoseStamped>(pose_in,pose_out,"odom",
               tf2::Duration(std::chrono::milliseconds(500)));

         pose_2d transform = ros2_pose_to_pose_2d(pose_out.pose);

         Points data = scan_to_points(msg,last_pos);

         map.add_points(data);
         fail_count = 0;
      }
      catch(const tf2::TransformException & ex) {
         RCLCPP_WARN(this->get_logger(),"transformation of scan failed for the %dth time: %s to %s",fail_count++,msg.header.frame_id.c_str(),"map");
         return;
      }
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
         Points data = cloud_to_points(msg, last_pos); 

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
      RCLCPP_INFO(this->get_logger(),"publishing map");
      nav_msgs::msg::OccupancyGrid msg;
      map.to_msg(msg);

      msg.info.map_load_time = this->get_clock()->now();
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "map";

      map_out->publish(msg);
   }

   // https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/TransformStamped.msg
   void broadcast_map_frame() {
      RCLCPP_INFO(this->get_logger(),"publishing map frame");
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
