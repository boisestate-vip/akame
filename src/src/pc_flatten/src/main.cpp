
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
#include <cmath>
#include <cstdint>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

std::vector<std::string> split(std::string & str, const std::string & delimiter);

struct point_data {
   float x;
   float y;
   float z;
};

struct bin_entry {
   float x;
   float y;
   int count;
};

struct low_pass_entry {
   int threshold;
   float bin_size;
};

class PcFlatten : public rclcpp::Node {
public:

   PcFlatten(): Node("pc_flatten") {

      /* everything is output as a point cloud */
      this->declare_parameter("cloud_out","/flattened");

      /* inputs to flatten and cull */
      this->declare_parameter("cloud_in","/points");
      this->declare_parameter("scan_in","/scan");

      /* special parameter for controlling a low pass filter over
       * sensor input. Format will be:
       *    link,point_threshold,bin_size
       * You can specify as many multiples of three as you would like.
       * The link identifies the sensor input to hit with the low pass.
       * The point_threshold is the number of points in a bin for it to
       * be counted as an obstacle. The bin_size is the size of each
       * bin in meters that points will be mapped into. Note that this
       * is a flattening along the z-axis, so bins are two dimensional. */
      this->declare_parameter("low_pass","synexens_link,25,0.025,"
                                         "single_point_link,2,0.05,");
      std::string low_pass_str = this->get_parameter("low_pass").as_string();
      collect_low_pass_entries(low_pass_str);

      /* reference frame to publish in */
      this->declare_parameter("frame_out","map");
      frame_out = this->get_parameter("frame_out").as_string();

      cloud_out = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("cloud_out").as_string(), 10);

      cloud_in = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("cloud_in").as_string(),
            10,std::bind(&PcFlatten::process_cloud, this, _1));
      scan_in = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->get_parameter("scan_in").as_string(),
            10,std::bind(&PcFlatten::process_scan, this, _1));

      tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

      /* Parameters for cropping and scan geometry */

      /* any points between the obs_threshold_low and
       * obj_threshold_high will be culled.           */
      this->declare_parameter("obs_threshold_low", -10.0);
      this->declare_parameter("obs_threshold_high",  0.1);
      obs_threshold_low = this->get_parameter("obs_threshold_low").as_double();
      obs_threshold_high = this->get_parameter("obs_threshold_high").as_double();
      
      /* cull obstacle points above this z value */
      this->declare_parameter("obs_ceiling",  0.6);
      obs_ceiling = this->get_parameter("obs_ceiling").as_double();

      /* whether to enter debug mode. In this mode, we publish
       * both the accepted and rejected point data. You can then
       * enter rviz2 and color them (red and green) to get a clear
       * idea of what is happening.                               */
      this->declare_parameter("debug",true);
      debug_mode = this->get_parameter("debug").as_bool();

      this->declare_parameter("debug_out","/flattened_rejected");
      debug_out = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("debug_out").as_string(), 10);

      this->declare_parameter("debug_lowpass_out","/lowpass_rejected");
      debug_lowpass_out = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("debug_lowpass_out").as_string(), 10);
   }

private:

   std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> cloud_in;

   std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> cloud_out;

   std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::LaserScan>> scan_in;

   std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> debug_out;
   std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> debug_lowpass_out;

   std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
   std::unique_ptr<tf2_ros::Buffer> tf_buffer;

   double obs_threshold_low, obs_threshold_high, obs_ceiling;
   std::string frame_out;
   bool debug_mode;

   std::unordered_map<std::string,low_pass_entry> filters;

   void collect_low_pass_entries(std::string & params) {

      auto splits = split(params,",");
      for (size_t i = 0; i+2 < splits.size(); i += 3) {

         std::string link = splits[i];
         std::string count_str = splits[i+1];
         std::string bin_size_str = splits[i+2];

         int threshold = (int)strtol(count_str.c_str(),NULL,0);
         float bin_size = (float)strtod(bin_size_str.c_str(),NULL);

         RCLCPP_INFO(this->get_logger(),"setting filter for %s: bin_size: %f, threshold: %d",
                     link.c_str(),bin_size,threshold);

         filters[link] = low_pass_entry{
            threshold, bin_size
         };
      }

   }

   float get_value(int size, uint64_t data) {
      switch (size) {
         case 1: {
            int8_t value = data; return (float)value; 
         } break;
         case 2: {
            int16_t value = data; return (float)value;
         } break;
         case 4: {
            uint32_t data_32 = data;
            float value;
            memcpy(&value,&data_32,4);
            return value;
         } break;
         case 8: {
            double value;
            memcpy(&value,&data,8);
            return (float)value;
         } break;
         default: return 0.0;
      }
   }


   void collect_points(const sensor_msgs::msg::PointCloud2 & msg,
                       std::vector<point_data> & points) {

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

            point_data new_point;

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

   void pc_to_ls(const sensor_msgs::msg::LaserScan & msg,
                 sensor_msgs::msg::PointCloud2 & cloud) {

      cloud.header = msg.header;

      sensor_msgs::msg::PointField field;

      field.name = "x";
      field.offset = 0;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      cloud.fields.push_back(field);

      field.name = "y";
      field.offset = 4;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      cloud.fields.push_back(field);

      field.name = "z";
      field.offset = 8;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      cloud.fields.push_back(field);

      cloud.is_bigendian = false;
      cloud.point_step = 12;

      cloud.height = 1;

      int count = 0;
      double angle = msg.angle_min - msg.angle_increment;
      for (size_t i = 0; i < msg.ranges.size(); ++i) {

         angle += msg.angle_increment;

         if (msg.ranges[i] < msg.range_min)
            continue; /* skip anything inside of range */

         /* these values are relative to the sensor */
         point_data point = {
            (float)(std::cos(angle) * std::min(msg.ranges[i], msg.range_max)),
            (float)(std::sin(angle) * std::min(msg.ranges[i], msg.range_max)),
            0.0,
         };

         uint8_t * as_bytes = (uint8_t *)&point;
         for (size_t i = 0; i < sizeof(point_data); ++i)
            cloud.data.push_back(as_bytes[i]);
         count += 1;
      }

      cloud.row_step = count;
      cloud.width = count;
   }

   int transform_cloud(const sensor_msgs::msg::PointCloud2 & in,
                       sensor_msgs::msg::PointCloud2 & out) {

      std::string frame_id = in.header.frame_id;

      out.header = in.header;
      out.header.frame_id = frame_out;

      geometry_msgs::msg::TransformStamped to_frame_out;

      /* TODO(*): verify that this will not cause ghost obstacles to occur */
      try {
         to_frame_out = tf_buffer->lookupTransform(
                                 frame_out, frame_id,
                                 tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
         RCLCPP_WARN(
            this->get_logger(), "Could not transform %s to %s: %s",
            frame_out.c_str(), frame_id.c_str(), ex.what());
            return 1;
      }

      /* transform the point cloud into the global reference frame */
      tf2::doTransform<sensor_msgs::msg::PointCloud2>(in,out,to_frame_out);

      return 0;
   }

   void points_to_cloud(const std::vector<point_data> & points,
                        sensor_msgs::msg::PointCloud2 & cloud) {
      sensor_msgs::msg::PointField field;

      field.name = "x";
      field.offset = 0;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      cloud.fields.push_back(field);

      field.name = "y";
      field.offset = 4;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      cloud.fields.push_back(field);

      field.name = "z";
      field.offset = 8;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      cloud.fields.push_back(field);

      cloud.is_bigendian = false;
      cloud.point_step = 12;
      cloud.row_step = points.size();

      cloud.height = 1;
      cloud.width = points.size();

      size_t byte_count = points.size()*sizeof(point_data);
      uint8_t * to_uints = (uint8_t *)points.data();
      cloud.data.reserve(byte_count);
      for (size_t i = 0; i < byte_count; ++i) {
         cloud.data.push_back(to_uints[i]);
      }
   }

   void cull_points(const std::vector<point_data> & in,
                    std::vector<point_data> & out) {
      out.reserve(in.size()); /* optimistic */

      point_data * pos = (point_data *)in.data();
      point_data * end = (point_data *)in.data() + in.size();

      for (; pos < end; pos += 1) {

         double z = pos->z;

         if ((z >= obs_threshold_low && z <= obs_threshold_high) ||
             z >= obs_ceiling) {
            continue;
         }

         out.push_back(*pos);
      }
   }

   void cull_points_debug(const std::vector<point_data> & in,
                          std::vector<point_data> & out,
                          std::vector<point_data> & reject) {
      out.reserve(in.size()); /* optimistic */

      point_data * pos = (point_data *)in.data();
      point_data * end = (point_data *)in.data() + in.size();

      for (; pos < end; pos += 1) {

         double z = pos->z;

         if ((z >= obs_threshold_low && z <= obs_threshold_high) ||
             z >= obs_ceiling) {
            reject.push_back(*pos);
            continue;
         }

         out.push_back(*pos);
      }
   }

   void lowpass(const low_pass_entry & filter,
                const std::vector<point_data> & in,
                std::vector<point_data> & out) {
      out.reserve(in.size());

      std::unordered_map<uint64_t,bin_entry> bins;

      point_data * pos = (point_data *)in.data();
      point_data * end = (point_data *)in.data() + in.size();

      for (; pos < end; pos += 1) {

         uint64_t key = ((uint32_t)(pos->x/filter.bin_size))|
                        (((uint64_t)(pos->y/filter.bin_size))<<32);

         if (bins.count(key))
            bins[key].count += 1;
         else bins[key] = bin_entry{pos->x,pos->y,1};
      }

      for (const std::pair<const uint64_t, bin_entry> & kv : bins) {
         if (kv.second.count >= filter.threshold) {
            out.push_back(point_data{kv.second.x,kv.second.y,0.0});
         }
      }
   }

   void lowpass_debug(const low_pass_entry & filter,
                      const std::vector<point_data> & in,
                      std::vector<point_data> & out,
                      std::vector<point_data> & reject) {
      out.reserve(in.size());

      std::unordered_map<uint64_t,bin_entry> bins;

      point_data * pos = (point_data *)in.data();
      point_data * end = (point_data *)in.data() + in.size();

      for (; pos < end; pos += 1) {

         uint64_t key = ((uint32_t)(pos->x/filter.bin_size))|
                        (((uint64_t)(pos->y/filter.bin_size))<<32);

         if (bins.count(key))
            bins[key].count += 1;
         else bins[key] = bin_entry{pos->x,pos->y,1};
      }

      for (const std::pair<const uint64_t, bin_entry> & kv : bins) {
         if (kv.second.count >= filter.threshold) {
            out.push_back(point_data{kv.second.x,kv.second.y,0.0});
         }
         else {
            reject.push_back(point_data{kv.second.x,kv.second.y,0.0});
         }
      }
   }

   void process_cloud(const sensor_msgs::msg::PointCloud2 & msg) {
   
      sensor_msgs::msg::PointCloud2 transformed;
      sensor_msgs::msg::PointCloud2 cloud;
      std::vector<point_data> points;
      std::vector<point_data> survivors;

      if (transform_cloud(msg,transformed))
         return;

      collect_points(transformed,points);

      if (debug_mode) {
         std::vector<point_data> rejected;
         sensor_msgs::msg::PointCloud2 rejected_cloud;

         cull_points_debug(points,survivors,rejected);


         if (filters.count(msg.header.frame_id)) {

            sensor_msgs::msg::PointCloud2 failed_cloud;
            std::vector<point_data> passed;
            std::vector<point_data> failed;

            lowpass_debug(filters[msg.header.frame_id],
                          survivors,passed,failed);

            points_to_cloud(passed,cloud);
            points_to_cloud(failed,failed_cloud);

            failed_cloud.header = transformed.header;
            debug_lowpass_out->publish(failed_cloud);
         }
         else {
            points_to_cloud(survivors,cloud);
         }

         points_to_cloud(rejected,rejected_cloud);
         cloud.header = transformed.header;
         rejected_cloud.header = transformed.header;

         cloud_out->publish(cloud);
         debug_out->publish(rejected_cloud);
      }
      else {
         cull_points(points,survivors);

         if (filters.count(msg.header.frame_id)) {
            std::vector<point_data> passed;

            lowpass(filters[msg.header.frame_id],survivors,passed);

            points_to_cloud(passed,cloud);
         }
         else {
            points_to_cloud(survivors,cloud);
         }
         cloud.header = transformed.header;

         cloud_out->publish(cloud);
      }
   }

   void process_scan(const sensor_msgs::msg::LaserScan & msg) {

      sensor_msgs::msg::PointCloud2 start;

      pc_to_ls(msg,start);

      process_cloud(start);
   }

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcFlatten>());
  rclcpp::shutdown();
  return 0;
}

std::vector<std::string> split(std::string & str, const std::string & delimiter) {
   std::vector<std::string> splits;
   size_t pos = 0;
   std::string token;
   
   while ((pos = str.find(delimiter)) != std::string::npos) {
      token = str.substr(0,pos);
      splits.push_back(token);
      str.erase(0,pos+delimiter.length());
   }
   splits.push_back(str);

   return splits;
}

