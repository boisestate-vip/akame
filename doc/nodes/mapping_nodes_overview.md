# Mapping Nodes Overview

NOTE TO TEAM: Feel free to delete this, but I included it just to make sure the 3 proposed mapping node documentations (graph_slam_node.md, map_odom_broadcaster.md, and pointcloud_to_laserscan_node.md) make sense... at least conceptually.

## Overview

This document summarizes the mapping slice of the navigation stack for Mapping Option A (2D Graph SLAM) from the PDR presentation. It explains how sensor drivers, localization, mapping nodes, and downstream planners fit together. 

The goal is to provide a clear, high-level view that matches the Google Draw navigation diagram and the individual node documents in this directory.

## Data flow (high level)

1. **Sensors**
   - IMU: VectorNav VN-200  
   - 2D LiDAR: YDLIDAR GS2  
   - 3D LiDAR / depth cameras: Unitree 4D Lidar L1, Intel RealSense D435i, Synexens CS20  
   - Global tracking: Vive 2 Tracker  

2. **Localization**
   - A robot_localization / Kalman filter node fuses IMU and wheel odometry (and possibly other sensors) to produce:
     - Filtered `nav_msgs/Odometry` in the `odom` frame  
     - TF `odom -> base_link`

3. **Mapping**
   - **PointCloud to LaserScan Node** (optional)  
     - Converts 3D `sensor_msgs/PointCloud2` data into synthetic 2D `sensor_msgs/LaserScan`.
   - **Graph SLAM Node**  
     - Ingests `LaserScan` and/or `PointCloud2` plus odometry and optional global pose.  
     - Builds and publishes a 2D `nav_msgs/OccupancyGrid` map in the `map` frame.  
     - Publishes the robot pose in `map` as `geometry_msgs/PoseStamped`.
   - **Map–Odom Broadcaster**  
     - Uses the Graph SLAM pose in `map` and the filtered odometry pose in `odom` to compute and broadcast `map -> odom`.

4. **Planning and control**
   - Path planners consume the `map` and TF tree (`map`, `odom`, `base_link`) to compute `nav_msgs/Path`.  
   - Path followers and controllers use the TF tree and path to compute `geometry_msgs/Twist` commands.  
   - The Differential Drive Controller converts `geometry_msgs/Twist` into left/right motor commands.

## Visual node mapping (Google Draw)

The following mapping-related boxes should appear in (or be connected into) the navigation diagram:

### Graph SLAM (2D Map)

- **ROS node:** Graph SLAM Node  
- **Inputs:**
  - From YDLIDAR GS2 (`sensor_msgs/LaserScan`)
  - From Unitree L1 / D435i / CS20 (`sensor_msgs/PointCloud2`) via the PointCloud → LaserScan Node or direct
  - From robot_localization (filtered `nav_msgs/Odometry` in `odom`)
  - From Vive 2 Tracker or similar (optional global odometry)
- **Outputs:**
  - `nav_msgs/OccupancyGrid` (`map`) to planners
  - `geometry_msgs/PoseStamped` (`graph_slam_pose`) to the Map–Odom Broadcaster
  - `visualization_msgs/MarkerArray` to RViz

### PointCloud → LaserScan (optional)

- **ROS node:** PointCloud to LaserScan Node  
- **Inputs:** `sensor_msgs/PointCloud2` from 3D sensors  
- **Output:** `sensor_msgs/LaserScan` to Graph SLAM Node

### Map–Odom Broadcaster

- **ROS node:** Map–Odom Broadcaster  
- **Inputs:**
  - `graph_slam_pose` from Graph SLAM Node
  - `nav_msgs/Odometry` from robot_localization
- **Output:**
  - TF `map -> odom` to the TF tree
