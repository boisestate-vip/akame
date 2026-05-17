
# pc_flatten

filter for all lidar sensors on the robot. Has a bunch of configurable
settings and stuff.

## usage

This node is three seperate filters that act in sequence on a point
cloud or laser scan, and return a point cloud that is the final result
of the filtering. The filters are as follows:

 * z-axis filter - reject points between a given z-axis range
 * lowpass filter - rejects points that don't appear together frequently enough
 * box filter - rejects points sitting within a box at the emitter

The lowpass filters are configured by setting a list inside of a ros parameter.
For the lowpass the list looks like:
```
<frame-id>,<threshold>,<bin-size>
```
Any number of these tuples can be given in the ``lowpass`` parameter. The bin
size controls the area, the threshold determines how many points have to appear
in a bin for the point to pass, and the frame id keys the filter on frame.
     
The box filters are configured in a similar manner:
```
<frame-id>,<box-size>
```
The difference is box-size, which specifies the area (in meters) that the filtering
box extends in each direction. This is also keyed on frame id.

## subscriptions

| type                           | description            |
|--------------------------------|------------------------|
| [geometry_msgs/PointCloud2][3] | point cloud to process |
| [geometry_msgs/LaserScan][2]   | laser scan to process  |

## publishers

| type                           | description                         |
|--------------------------------|-------------------------------------|
| [geometry_msgs/PointCloud2][3] | final filtered cloud output         |
| [geometry_msgs/PointCloud2][3] | z range culled points in debug mode |
| [geometry_msgs/PointCloud2][3] | lowpass culled points in debug mode |
| [geometry_msgs/PointCloud2][3] | box culled points in debug mode     |

## parameters

| name               | description                                                                               | type   |
|--------------------|-------------------------------------------------------------------------------------------|--------|
| lowpass            | lowpass filter params. A list of csv values with the form ``frame_id,threshold,bin_size`` | string |
| inner_box          | inner box filter params. A list of csv values with the form ``frame_id,radius``           | string |
| frame_out          | frame to transform output to                                                              | string |
| cloud_out          | topic to publish final filtered data on                                                   | string |
| cloud_in           | topic to recieve point cloud data on                                                      | string |
| scan_in            | topic to recieve scan on                                                                  | string |
| obs_threshold_low  | points with a z inside of this and obs_threshold_high are culled                          | float  |
| obs_threshold_high | points with a z inside of this and obs_threshold_low are culled                           | float  |
| obs_ceiling        | maximum height at which points are accepted                                               | float  |
| debug              | also publish debug topic nodes                                                            | bool   |
| debug_out          | topic to publish points rejected from the z filter on                                     | string |
| debug_lowpass_out  | topic to publish points rejected from the lowpass filter on                               | string |
| debug_box_out      | topic to publish points rejected from the box filter on                                   | string |

## transforms

This node expects to be able to transform from the frame it recieves sensors
on to the frame that is set in the parameter frame_out. This transform should
be doable. Additionally, the frames of inputs are used to flag the lowpass and
box filters.

## _

[1]: https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg
[2]: https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg
[3]: https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg
[4]: https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/Twist.msg
[5]: https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/OccupancyGrid.msg
[6]: https://github.com/ros2/common_interfaces/blob/rolling/visualization_msgs/msg/MarkerArray.msg
[7]: https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/TransformStamped.msg
[8]: https://github.com/ros2/common_interfaces/blob/rolling/geometry_msgs/msg/PoseStamped.msg
[9]: https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Path.msg
[10]: https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/Empty.msg
[11]: https://github.com/ros2/common_interfaces/blob/rolling/visualization_msgs/msg/Marker.msg
