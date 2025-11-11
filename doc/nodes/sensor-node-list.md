
This document serves as a quick reference for the sensors used in this project. PLEASE DOUBLE CHECK I DIDN'T GET THE PUBLISHED TOPICS WRONG, ty

## 1. Vive 2 Tracker

- **Description**: The Vive 2 Tracker is a motion tracking device that can be used to track the position and orientation of the reciever in 3D space.
- **GitHub Repository**: [Vive 2 Tracker](https://github.com/Flypulator/vive_tracker_ros2)
- **Published Topics**:
  - `/vive_tracker_pose`: **[geometry_msgs/PoseWithCovarianceStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)**
  - `/vive_tracker_odom`: **[geometry_msgs/Odometry](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)**

## 2. VectorNav VN-200

- **Description**: The VectorNav VN-200 is a high-performance IMU (Inertial Measurement Unit) and INS (Inertial Navigation System) unit. It provides precise 3D orientation, velocity, and position data.
- **GitHub Repository**: [VectorNav VN-200 IMU](https://github.com/dawonn/vectornav)
- **Published Topics**:
  - `/vectornav/IMU`: **[sensor_msgs/Imu](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)**
  - `/vectornav/Odom`: **[nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)**
  - `/vectornav/INS`: **[vectornav/Ins](https://github.com/dawonn/vectornav/blob/master/msg/Ins.msg)**


## 3. YDLIDAR GS2

- **Description**: The YDLIDAR GS2 is a 360-degree flat laser scanner used to capture 2D LiDAR data.
- **GitHub Repository**: [YDLIDAR GS2 Lidar](https://github.com/YDLIDAR/ydlidar_ros2_driver)
- **Published Topics**:
  - `/scan`: **[sensor_msgs/LaserScan](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html)**

## 4. Unilidar 4D L1

- **Description**: The Unilidar 4D L1 is a 3D half-sphere LiDAR sensor.
- **GitHub Repository**: [Unilidar 4D L1 Lidar](https://github.com/unitreerobotics/unilidar_sdk2?tab=readme-ov-file)
- **Published Topics**:
  - `/unilidar/scan`: **[unilidar/cloud](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)**
  - `/unilidar/imu`: **[unilidar/imu](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)**

## 5. Synexens CS20

- **Description**: The Synexens CS20 is a 3D LiDAR sensor that captures a cone in front of it.
- **GitHub Repository**: [Synexens CS20 Lidar](https://github.com/qeftser/synexens_iron)
- **Published Topics**:
  - `/synexens_cs20/scan`: **[sensor_msgs/LaserScan](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html)**
