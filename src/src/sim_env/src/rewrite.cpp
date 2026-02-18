
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

/* this is the actual ros node. The class is the node. */
class SimRewrite : public rclcpp::Node {
public:

   SimRewrite() : Node("sim_env_rewrite") {

      /* now instantiate our subscriptions to our information sources */
      scan_in = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SimRewrite::rewrite_scan, this, _1));
      scan_out = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/demo/scan",10);

      imu_in = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&SimRewrite::rewrite_imu, this, _1));
      imu_out = this->create_publisher<sensor_msgs::msg::Imu>(
            "/demo/imu",10);

      scan_points_in = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/scan/points", 10,
            std::bind(&SimRewrite::rewrite_scan_points, this, _1));
      scan_points_out = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/demo/scan/points",10);

      depth_camera_points_in = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/depth_camera/points", 10,
            std::bind(&SimRewrite::rewrite_depth_camera_points, this, _1));
      depth_camera_points_out = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/demo/depth_camera/points",10);


   }

private:

   /* === node variables === */

   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_in;
   rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_out;

   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_in;
   rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_out;

   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_points_in;
   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_points_out;

   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_camera_points_in;
   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_camera_points_out;

   /* === node callback handlers === */

   void rewrite_scan(const sensor_msgs::msg::LaserScan & msg) {
      auto new_msg = msg;
      new_msg.header.frame_id = "lidar_link";
      scan_out->publish(new_msg);
   }

   void rewrite_imu(const sensor_msgs::msg::Imu & msg) {
      auto new_msg = msg;
      new_msg.header.frame_id = "imu_link";
      imu_out->publish(new_msg);
   }

   void rewrite_scan_points(const sensor_msgs::msg::PointCloud2 & msg) {
      auto new_msg = msg;
      new_msg.header.frame_id = "lidar_link";
      scan_points_out->publish(new_msg);
   }

   void rewrite_depth_camera_points(const sensor_msgs::msg::PointCloud2 & msg) {
      auto new_msg = msg;
      new_msg.header.frame_id = "camera_link";
      depth_camera_points_out->publish(new_msg);
   }

};

int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<SimRewrite>());
   rclcpp::shutdown();
   return 0;
}
