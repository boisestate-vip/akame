#include <chrono>
#include <memory>
#include <string>
#include <functional>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>  /* for std::sqrt, std::atan2, std::fabs */

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

struct point3 {
   double x;
   double y;
   double z;
};

struct point2 {
   double x;
   double y;
};

class PcToLs : public rclcpp::Node {

public:

   /* constructor */
   PcToLs() : Node("pc_to_ls") {

      cloud_out = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_out", 10);
      /* LaserScan output topic */
      scan_out = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_out", 10);

      cloud_in = this->create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_in",10,std::bind(&PcToLs::process_cloud, this, _1));
      /* Optional: LaserScan -> PointCloud path */
      scan_in = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan_in",10,std::bind(&PcToLs::process_scan, this, _1));

      tf_buffer =
         std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener =
         std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

      /* Parameters for cropping and scan geometry */

      this->declare_parameter<double>("cull_range", 0.0);  // 0.0 = disabled

      this->declare_parameter<double>("min_height", -0.1);
      this->declare_parameter<double>("max_height",  0.1);

      this->declare_parameter<double>("angle_min", -3.14);
      this->declare_parameter<double>("angle_max", 3.14);
      this->declare_parameter<double>("angle_increment", 0.01);

      this->declare_parameter<double>("range_min", 0.05);
      this->declare_parameter<double>("range_max", 20.0);

   }

private:

   std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> cloud_in;
   std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> cloud_out;

   /* LaserScan publisher (PointCloud -> LaserScan) */
   std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> scan_out;

   /* Optional: LaserScan subscriber (LaserScan -> PointCloud) */
   std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::LaserScan>> scan_in;

   std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
   std::unique_ptr<tf2_ros::Buffer> tf_buffer;

   double get_value(int size, uint64_t data) {

      switch (size) {

         case 1:
            {
               int8_t value = data;
               return (double)value; 
            }
            break;

         case 2:
            {
               int16_t value = data;
               return (double)value;
            }
            break;

         case 4:
            {
               uint32_t data_32 = data;
               float value;
               memcpy(&value,&data_32,4);
               return (double)value;
            }
            break;
            
         case 8:
            {
               double value;
               memcpy(&value,&data,8);
               return value;
            }
            break;
            
         default:
            return 0.0;
            break;
      }
   }


   void collect_points(std::vector<point3> & points, const sensor_msgs::msg::PointCloud2 & msg) {

      int entries = (msg.width * msg.height) / msg.point_step;

      int x_off = -1, x_size;
      int y_off = -1, y_size;
      int z_off = -1, z_size;

      for (const sensor_msgs::msg::PointField & field : msg.fields) {
         if (field.name == "x") {
            x_off = field.offset;
            x_size = (field.datatype < 3 ? 1 : (field.datatype < 5 ? 2 : (field.datatype < 8 ? 4 : 8)));
         }
         else if (field.name == "y") {
            y_off = field.offset;
            y_size = (field.datatype < 3 ? 1 : (field.datatype < 5 ? 2 : (field.datatype < 8 ? 4 : 8)));
         }
         else if (field.name == "z") {
            z_off = field.offset;
            z_size = (field.datatype < 3 ? 1 : (field.datatype < 5 ? 2 : (field.datatype < 8 ? 4 : 8)));
         }
      }

      /* we have all the data fields we expect, proceed normally */
      if (x_off >= 0 && y_off >= 0 && z_off >= 0) {
         for (size_t i = 0; i < msg.data.size(); i += msg.point_step) {

            point3 new_point;

            uint64_t data;

            memcpy(&data,&msg.data[i+x_off],x_size); /* x value */
            new_point.x = get_value(x_size,data);
            memcpy(&data,&msg.data[i+y_off],y_size); /* y value */
            new_point.y = get_value(y_size,data);
            memcpy(&data,&msg.data[i+z_off],z_size); /* z value */
            new_point.z = get_value(z_size,data);

            points.push_back(new_point);

         }
      }
      else {
         RCLCPP_WARN(this->get_logger(),
                     "Failed to find x,y,z data in point cloud: %s",
                     msg.header.frame_id.c_str());
            return;
      }
   }

   void process_cloud(const sensor_msgs::msg::PointCloud2 & msg) {
   
      sensor_msgs::msg::PointCloud2 msg_transformed;
      sensor_msgs::msg::PointCloud2 msg_2d;

      std::string frame_id = msg.header.frame_id;
      std::string new_frame_id = "map";

      msg_2d.header = msg.header;
      msg_2d.header.frame_id = new_frame_id;
   
      geometry_msgs::msg::TransformStamped to_map;

      /* TODO(*): verify that this will not cause ghost obstacles to occur */
      try {
         to_map = tf_buffer->lookupTransform(
                                 new_frame_id, frame_id,
                                 tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
         RCLCPP_WARN(
            this->get_logger(), "Could not transform %s to %s: %s",
            new_frame_id.c_str(), frame_id.c_str(), ex.what());
            return;
      }

      /* transform the point cloud into the global reference frame */
      tf2::doTransform<sensor_msgs::msg::PointCloud2>(msg,msg_transformed,to_map);

      /* go through the point cloud and get rid of any points we don't want. */
      std::vector<point3> points;
      std::vector<point2> points_out;
      collect_points(points,msg_transformed);

      /* TODO(*): make this a parameter */
      // double cull_range = 0.1;
      double min_height = this->get_parameter("min_height").as_double();
      double max_height = this->get_parameter("max_height").as_double();
      double cull_range = this->get_parameter("cull_range").as_double();
      for (point3 & point : points) {

         // 1) Keep only points inside the slice
         if (point.z < min_height || point.z > max_height)
            continue;

         // 2) Optionally drop a thin band around z = 0
         if (cull_range > 0.0 && std::fabs(point.z) < cull_range)
            continue;

         // if (point.z < min_height || point.z > max_height)
         //    continue;
         // double dist = point.z;
         // if (std::fabs(dist) < cull_range)
         //    continue;
         points_out.push_back(point2{ point.x, point.y });
         // points_out.push_back((point2){.x=point.x,.y=point.y});
      }

      msg_2d.width = points_out.size();
      msg_2d.height = 1;

      sensor_msgs::msg::PointField x_field;
      sensor_msgs::msg::PointField y_field;

      x_field.name = "x";
      x_field.offset = 0;
      x_field.datatype = 8; /* FLOAT64 */
      x_field.count = 1;

      y_field.name = "y";
      y_field.offset = 8;
      y_field.datatype = 8; /* FLOAT64 */
      y_field.count = 1;

      msg_2d.fields.push_back(x_field);
      msg_2d.fields.push_back(y_field);

      msg_2d.is_bigendian = false;
      msg_2d.point_step = 16; /* two 8-byte floats */
      msg_2d.row_step = msg_2d.width * msg_2d.point_step;
      msg_2d.is_dense = true;

      size_t data_size = sizeof(uint8_t)*points_out.size()*msg_2d.point_step;
      uint8_t * data = (uint8_t *)malloc(data_size);
      int offset = 0;
      for (point2 & point : points_out) {

         /* 8 is the size of our double type */
         memcpy(&data[offset+0],&point.x,8);
         memcpy(&data[offset+8],&point.y,8);

         offset += msg_2d.point_step;
      }

      for (size_t i = 0; i < data_size; ++i)
         msg_2d.data.push_back(data[i]);

      cloud_out->publish(msg_2d);

      /* Build LaserScan message from 2D points (PointCloud -> LaserScan) */

      double angle_min       = this->get_parameter("angle_min").as_double();
      double angle_max       = this->get_parameter("angle_max").as_double();
      double angle_increment = this->get_parameter("angle_increment").as_double();
      double range_min       = this->get_parameter("range_min").as_double();
      double range_max       = this->get_parameter("range_max").as_double();

      /* LaserScan geometry */
      int num_bins = (int)((angle_max - angle_min) / angle_increment);

      if (num_bins <= 0) {
         RCLCPP_WARN(this->get_logger(),"PcToLs: invalid LaserScan geometry (num_bins <= 0)");
         return;
      }

      sensor_msgs::msg::LaserScan scan;

      scan.header = msg_2d.header;
      scan.header.frame_id = new_frame_id;

      scan.angle_min       = angle_min;
      scan.angle_max       = angle_max;
      scan.angle_increment = angle_increment;
      scan.range_min       = range_min;
      scan.range_max       = range_max;

      scan.time_increment  = 0.0;
      scan.scan_time       = 0.0;

      scan.ranges.assign(num_bins, std::numeric_limits<float>::infinity());
      scan.intensities.assign(num_bins, 0.0f);

      for (point2 & point : points_out) {

         double r = std::sqrt(point.x*point.x + point.y*point.y);
         if (r < range_min || r > range_max)
            continue;

         double theta = std::atan2(point.y, point.x);

         if (theta < angle_min || theta > angle_max)
            continue;

         int index = (int)((theta - angle_min) / angle_increment);

         if (index < 0 || index >= num_bins)
            continue;

         float r_f = (float)r;
         if (r_f < scan.ranges[index]) {
            scan.ranges[index] = r_f;
            /* intensities[index] can stay 0 for now */
         }
      }

      scan_out->publish(scan);
   }

   void process_scan(const sensor_msgs::msg::LaserScan & msg) {

      /* Convert LaserScan into 3D points in the scan frame (z=0) */

      int entries = (int)((msg.angle_max - msg.angle_min) / msg.angle_increment);
      if (entries <= 0) {
         RCLCPP_WARN(this->get_logger(),"PcToLs: invalid LaserScan geometry (entries <= 0)");
         return;
      }

      std::vector<point3> points_scan;
      points_scan.reserve(entries);

      for (int i = 0; i < entries; ++i) {

         if (i >= (int)msg.ranges.size())
            break;

         double r = msg.ranges[i];

         if (r < msg.range_min || r > msg.range_max)
            continue;

         /* LaserScan to points, serves as our "doTransform" */
         double angle = msg.angle_min + msg.angle_increment*i;
         double x = r * std::cos(angle);
         double y = r * std::sin(angle);

         points_scan.push_back((point3){.x=x,.y=y,.z=0.0});
      }

      if (points_scan.empty()) {
         return;
      }

      /* Look up transform from scan frame to target_frame (e.g. map) */
      std::string target_frame = "map";   /* could also be a parameter */
      std::string scan_frame   = msg.header.frame_id;

      /* Transform points from scan frame into target_frame */
      geometry_msgs::msg::TransformStamped transform;
      try {
         transform = tf_buffer->lookupTransform(
            target_frame,
            scan_frame,
            tf2::TimePointZero);
      }
      catch (tf2::TransformException & ex) {
         RCLCPP_WARN(this->get_logger(),"PcToLs: %s",ex.what());
         return;
      }

      tf2::Transform tf;
      tf2::fromMsg(transform.transform, tf);

      /* Apply transform to each point (LaserScan -> target frame) */
      std::vector<point3> points_out;
      points_out.reserve(points_scan.size());

      for (point3 & point : points_scan) {

         tf2::Vector3 p(point.x, point.y, point.z);
         tf2::Vector3 p_t = tf * p;

         points_out.push_back((point3){
            .x = p_t.x(),
            .y = p_t.y(),
            .z = p_t.z()
         });
      }

      /* Pack into a PointCloud2, similar to process_cloud’s msg_2d */

      sensor_msgs::msg::PointCloud2 cloud_msg;

      cloud_msg.header = msg.header;
      cloud_msg.header.frame_id = target_frame;

      cloud_msg.width  = points_out.size();
      cloud_msg.height = 1;

      sensor_msgs::msg::PointField field_x;
      field_x.name = "x";
      field_x.offset = 0;
      field_x.datatype = sensor_msgs::msg::PointField::FLOAT64;
      field_x.count = 1;

      sensor_msgs::msg::PointField field_y;
      field_y.name = "y";
      field_y.offset = 8;
      field_y.datatype = sensor_msgs::msg::PointField::FLOAT64;
      field_y.count = 1;

      sensor_msgs::msg::PointField field_z;
      field_z.name = "z";
      field_z.offset = 16;
      field_z.datatype = sensor_msgs::msg::PointField::FLOAT64;
      field_z.count = 1;

      cloud_msg.fields.clear();
      cloud_msg.fields.push_back(field_x);
      cloud_msg.fields.push_back(field_y);
      cloud_msg.fields.push_back(field_z);

      cloud_msg.is_bigendian = false;
      cloud_msg.point_step   = 24;
      cloud_msg.row_step     = cloud_msg.point_step * cloud_msg.width;
      cloud_msg.is_dense     = true;

      cloud_msg.data.resize(cloud_msg.row_step);

      for (std::size_t i = 0; i < points_out.size(); ++i) {

         const point3 & p = points_out[i];

         std::size_t offset = i*cloud_msg.point_step;

         std::memcpy(&cloud_msg.data[offset+0], &p.x, 8);
         std::memcpy(&cloud_msg.data[offset+8], &p.y, 8);
         std::memcpy(&cloud_msg.data[offset+16],&p.z, 8);
      }

      cloud_out->publish(cloud_msg);
   }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcToLs>());
  rclcpp::shutdown();
  return 0;
}
