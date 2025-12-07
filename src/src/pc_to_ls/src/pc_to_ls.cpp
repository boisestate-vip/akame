
#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

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
      cloud_in = this->create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_in",10,std::bind(&PcToLs::process_cloud, this, _1));

      tf_buffer =
         std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener =
         std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

   }

private:

   std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> cloud_in;
   std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> cloud_out;

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
      double cull_range = 0.1;
      for (point3 & point : points) {

         double dist = point.z;
         if (std::fabs(dist) < cull_range)
            continue;

         points_out.push_back((point2){.x=point.x,.y=point.y});
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
   }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcToLs>());
  rclcpp::shutdown();
  return 0;
}
