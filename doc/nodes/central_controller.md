# Central Controller (Brain)

## Overview
This node acts as the high-level controller of the entire system. It generates the goal pose, controls when the robot is digging, dumping, or navigating, and tells the regolith node where to position the drum.

(Note: Update the link to the path generating node and figure out how to output regolith messages.)

## Inputs

| Name        | Type                                | Description                                                      |
|-------------|-------------------------------------|------------------------------------------------------------------|
| map       | [nav_msgs/OccupancyGrid](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/OccupancyGrid.msg)           | Occupancy grid representing the map, used to pick the goal pose. |
| pose_in   | [geometry_msgs/PoseStamped](https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/PoseStamped.msg)       | Standard pose of the robot. |

## Outputs

| Name                | Type                               | Description                                                  |
|---------------------|------------------------------------|--------------------------------------------------------------|
| goal_pose         | [geometry_msgs/PoseStamped](https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/PoseStamped.msg)       | Used to generate a path in the path generator node. |
| regolith_arm_angle| [std_msgs/Float64](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html)                 | Sets the height of the regolith arm. |
| regolith_drum_speed| [std_msgs/Float64](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html)               | Sets the speed of the regolith drum. |
