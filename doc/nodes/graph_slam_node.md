# Graph SLAM Node

## Overview

This node is the primary 2D Graph SLAM backend for the navigation stack (Option A).

It maintains a 2D pose graph, produces a `nav_msgs/OccupancyGrid` map in the `map` frame, and publishes the robot pose in `map` for use by planners and other nodes. It does not directly publish the `map -> odom` transform; that responsibility is delegated to the Map–Odom Broadcaster node.

## Inputs

| name       | msg type                       | description                                                                                       |
|------------|--------------------------------|---------------------------------------------------------------------------------------------------|
| scan       | [sensor_msgs/LaserScan][1]     | 2D laser scan data from a planar LiDAR (e.g. YDLIDAR GS2)                                        |
| cloud_in   | [sensor_msgs/PointCloud2][2]   | (Optional) 3D point cloud data from a depth camera or 3D LiDAR, if used for mapping             |
| odom_in    | [nav_msgs/Odometry][3]         | Filtered odometry in the `odom` frame from the localization node(s)                             |
| beacon_in  | [nav_msgs/Odometry][3]         | (Optional) Global pose/odometry source (e.g. Vive tracker, GNSS). Used as a stronger reference  |
| cmd_vel    | [geometry_msgs/Twist][4]       | (Optional) Commanded base velocity, used when integrating velocity directly                      |
| reset_graph| [std_msgs/Empty][5]            | (Optional) Signal to clear or reinitialize the pose graph and map                               |

## Outputs

| name               | msg type                            | description                                                                                 |
|--------------------|--------------------------------------|---------------------------------------------------------------------------------------------|
| map                | [nav_msgs/OccupancyGrid][6]          | 2D occupancy grid map in the `map` frame                                                    |
| graph_slam_pose    | [geometry_msgs/PoseStamped][7]       | Robot pose estimate in the `map` frame (derived from the last optimized graph node)        |
| graph_slam_markers | [visualization_msgs/MarkerArray][8]  | Optional visualization of the pose graph, constraints, and keyframes for RViz               |

## Implementation

This node wraps a 2D Graph SLAM backend implemented in C++ using a pose graph and occupancy grid mapping.

High-level behavior:

- Ingests odometry-like inputs (`odom_in`, optionally `beacon_in`) along with range measurements (`scan`, optionally `cloud_in`) and packages them into “observations”.
- For each observation, chooses the best input pose to anchor that observation:
  - If a fresh `beacon_in` message is available (within a configured timeout), use the beacon pose as the global reference.
  - Otherwise, use the `odom_in` pose transformed into the current `map` frame using the last `map -> odom` estimate.
- Optionally integrates motion between observations using either:
  - The twist fields in `odom_in` (if `use_odom_vel` is true), or
  - The separate `cmd_vel` topic (if `use_odom_vel` is false).
- Inserts observations into the graph-SLAM backend, which:
  - Decides when to add new nodes based on linear and angular displacement thresholds.
  - Performs scan / point cloud matching to nearby nodes.
  - Adds constraints and periodically optimizes the pose graph.
- Builds and maintains a 2D occupancy grid map in the `map` frame using an inverse sensor model and log-odds updates.
- Publishes:
  - The current occupancy grid map on the `map` topic.
  - The robot’s pose in the `map` frame as `graph_slam_pose`.
  - Optional RViz visualization markers as `graph_slam_markers`.

This node focuses solely on SLAM and map generation. It exposes the optimized robot pose in `map` to other nodes as an explicit topic and leaves computation of `map -> odom` to the dedicated Map–Odom Broadcaster node.

## Parameters

Exact parameter values will be set in configuration files, but the main categories are:

### Frontend / observation parameters

- `scan_in`, `cloud_in`, `odom_in`, `beacon_in`, `vel_in`, `reset_in`  
  Topic names for the LaserScan, PointCloud2, odometry, beacon, velocity, and reset inputs.
- `aggregate_sensor_data`, `aggregation_interval`  
  Control whether individual sensor messages are aggregated into a single combined observation and how often that aggregated observation is flushed into the SLAM backend.
- `use_odom_vel`  
  Selects whether the motion model uses the twist fields from `odom_in` or the separate `cmd_vel` topic.
- `estimate_movement_updates`  
  Enables or disables integrating the motion model between discrete measurements.
- `time_error`  
  Allowed time difference between sensor measurements and odometry when building an observation.
- `beacon_lost_time`  
  Time threshold for treating `beacon_in` as stale; beyond this, the node falls back to odom-based pose.
- `fix_beacon_nodes`  
  Controls whether graph nodes associated with beacon-based poses are treated as fixed anchors.

### Graph-SLAM backend parameters

- `algorithm`  
  SLAM backend selection (e.g., `"builtin"` for the default implementation).
- `bin_size`  
  Spatial binning size for organizing graph nodes and accelerating nearest-neighbor lookups.
- `linear_update_dist`, `angular_update_dist`  
  Minimum translation and rotation required before adding a new node to the graph.
- `lidar_acceptance_threshold`, `point_cloud_acceptance_threshold`  
  Matching quality thresholds used to accept or reject scan / point cloud matches.
- `node_association_dist`  
  Distance threshold for associating new observations with existing nodes.
- `recent_length`  
  Number of recent nodes considered during local optimization and loop closure checks.
- `loop_closure_dist`  
  Distance threshold for triggering loop closure checks between non-adjacent nodes.

### Map publication parameters

- `map_out`  
  Topic name for the `nav_msgs/OccupancyGrid` output.
- `map_publish_interval`  
  Interval (seconds) used for map and visualization publishing.

## Visual node (Google Draw)

Label the box: **“Graph SLAM (2D Map)”**.

Connect:

- Incoming arrows from:
  - Sensor boxes that produce `LaserScan` or `PointCloud2` (e.g., YDLIDAR GS2, Unitree 4D Lidar L1, Intel RealSense D435i, Synexens CS20).
  - The **robot_localization / Kalman Filter** box (for `odom_in`).
  - The Vive / global tracking box (for `beacon_in`, if used).
- Outgoing arrows to:
  - A “Map” / planner box (consumes `map`).
  - The **Map–Odom Broadcaster** box (consumes `graph_slam_pose`).
  - RViz / visualization (consumes `graph_slam_markers`).

[1]: https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg  
[2]: https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg  
[3]: https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg  
[4]: https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/Twist.msg  
[5]: https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/Empty.msg  
[6]: https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/OccupancyGrid.msg  
[7]: https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/PoseStamped.msg  
[8]: https://github.com/ros2/common_interfaces/blob/rolling/visualization_msgs/msg/MarkerArray.msg
