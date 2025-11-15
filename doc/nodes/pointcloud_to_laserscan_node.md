# PointCloud to LaserScan Node

## Overview

This node converts 3D point cloud data from a depth camera or 3D LiDAR into a synthetic 2D [`sensor_msgs/LaserScan`][2] topic for use by SLAM backends that expect planar scans.

It allows the Graph SLAM Node to consume LaserScan-like data even when the underlying sensor is a 3D sensor.

## Inputs

| name      | msg type                      | description                                                                                 |
|-----------|------------------------------|---------------------------------------------------------------------------------------------|
| cloud_in  | [sensor_msgs/PointCloud2][1] | 3D point cloud data (e.g. from Unitree L1, RealSense D435i, or Synexens CS20)              |
| tf_frames | TF transforms                | TF data used to transform the point cloud into the target scan frame                        |

## Outputs

| name     | msg type                  | description                                                      |
|----------|---------------------------|------------------------------------------------------------------|
| scan_out | [sensor_msgs/LaserScan][2]| 2D laser scan approximation generated from the incoming point cloud |

## Implementation

This node is expected to use the standard `pointcloud_to_laserscan` pattern.

High-level behavior:

- Transforms the incoming point cloud into a configured target frame representing the desired LiDAR plane.
- Filters points by height and range to select points near a horizontal slice.
- Projects the selected points into a 2D `LaserScan` message with configured angle and range limits.
- Publishes the resulting synthetic scan on a topic that the Graph SLAM Node subscribes to as its `scan` input.

Key parameters (set in configuration files based on mounting and use case) include:

- `target_frame`
- `min_height` and `max_height`
- `angle_min`, `angle_max`, `angle_increment`
- `range_min` and `range_max`

If we are not using 3D sensors for mapping in a particular configuration, this node can be disabled, and the SLAM backend can subscribe directly to a physical 2D LiDAR topic instead.

## Diagram placement (Google Draw)

Label the box: **“PointCloud → LaserScan”**.

Connect:

- Incoming arrows from:
  - 3D sensor boxes that publish `sensor_msgs/PointCloud2` (e.g., Unitree 4D Lidar L1, Intel RealSense D435i, Synexens CS20).
- Outgoing arrow to:
  - The **Graph SLAM (2D Map)** box, feeding its `scan` input.

[1]: https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg  
[2]: https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg
