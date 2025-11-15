# PointCloud to LaserScan Node

## Overview

This node converts 3D point cloud data from a depth camera or 3D LiDAR into a synthetic 2D `sensor_msgs/LaserScan` topic for use by SLAM backends that expect planar scans.

It allows the Graph SLAM Node to consume LaserScan-like data even when the underlying sensor is a 3D sensor.

## Inputs

| name      | msg type                        | description                                                                                  |
|-----------|----------------------------------|----------------------------------------------------------------------------------------------|
| cloud_in  | [sensor_msgs/PointCloud2][1]     | 3D point cloud data (e.g. from Unitree L1, RealSense D435i, or Synexens CS20)              |
| tf_frames | TF transforms                    | TF data used to transform the point cloud into the target scan frame                        |

## Outputs

| name     | msg type                    | description                                                       |
|----------|-----------------------------|-------------------------------------------------------------------|
| scan_out | [sensor_msgs/LaserScan][2]  | 2D laser scan approximation generated from the incoming point cloud |

## Implementation

This node is expected to use a standard `pointcloud_to_laserscan` pattern. I belive that in typical launch files, scan_out will be remapped to the Graph SLAM Node’s scan input topic (e.g. scan).

High-level behavior:

- Transforms the incoming point cloud into a configured target frame representing the desired LiDAR plane.
- Filters points by height and range to select points near a horizontal slice.
- Projects the selected points into a 2D `LaserScan` message with configured angle and range limits.
- Publishes the resulting synthetic scan on a topic that the Graph SLAM Node subscribes to as its scan input.

Parameters such as:

- `target_frame`
- `min_height` and `max_height`
- `angle_min`, `angle_max`, `angle_increment`
- `range_min` and `range_max`

will be set in configuration files based on the specific sensor mounting and use case.

If we are not using 3D sensors for mapping in a particular configuration, this node can be disabled, and the SLAM backend can subscribe directly to a physical 2D LiDAR topic instead.

[1]: https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg  
[2]: https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg
