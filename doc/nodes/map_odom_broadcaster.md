# Map–Odom Broadcaster

## Overview

This node computes and publishes the `map -> odom` transform so that the rest of the system can treat `odom` as a locally smooth motion frame while still being anchored to the global `map` frame.

It combines a robot pose in `map` (from the Graph SLAM Node) with a robot pose in `odom` (from the localization node(s)) and solves for the transform that makes them consistent. This is where we implement a non-identity `map -> odom` for Option A.

## Inputs

| name            | msg type                        | description                                                                                 |
|-----------------|----------------------------------|---------------------------------------------------------------------------------------------|
| graph_slam_pose | [geometry_msgs/PoseStamped][1]  | Robot pose estimate in the `map` frame from the Graph SLAM Node                             |
| odom_pose       | [nav_msgs/Odometry][2]          | Filtered robot pose estimate in the `odom` frame from the localization node(s)             |

## Outputs

| name          | msg type     | description                                           |
|---------------|-------------|-------------------------------------------------------|
| map_to_odom_tf| TF transform| Broadcast of the `map -> odom` transform on `/tf`    |

## Implementation

On startup, this node waits until it has received at least one valid pose in both the `map` and `odom` frames.

When both poses are available and time-synchronized closely enough:

- It computes the rigid-body transform \(T_{\text{map\_odom}}\) such that  
  \(T_{\text{map\_odom}} \cdot T_{\text{odom\_base\_link}} = T_{\text{map\_base\_link}}\).
- Here:
  - \(T_{\text{odom\_base\_link}}\) comes from the `odom_pose` input.
  - \(T_{\text{map\_base\_link}}\) comes from `graph_slam_pose`.

The node then:

- Publishes `map -> odom` on `/tf` at a fixed rate or whenever a new synchronized pose pair is available.
- Keeps the last valid transform when one of the inputs temporarily stops updating, and may log a warning if the data is stale beyond a configured timeout.

This node owns the `map -> odom` transform for Option A. Mapping and planning code should assume that:

- `odom -> base_link` is provided by the localization node(s).
- `map -> odom` is provided by this node.
- `map -> base_link` is available from the TF tree as a composed transform.

Exact synchronization behavior, timeouts, and `frame_id` values will be finalized in the node’s configuration and launch files.

## Visual node (Google Draw)

Label the box: **“Map–Odom Broadcaster”**.

Connect:

- Incoming arrows from:
  - **Graph SLAM (2D Map)** box (`graph_slam_pose`).
  - **robot_localization / Kalman Filter** box (`odom_pose`).
- Outgoing arrow to:
  - A TF label or a generic “TF tree” annotation feeding planners and the Differential Drive Controller (which rely on consistent `map`, `odom`, and `base_link` frames).

[1]: https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/PoseStamped.msg  
[2]: https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg
