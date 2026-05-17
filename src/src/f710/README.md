
# f710

used for manually driving the robot.

## setup

You will need to do some extra work to get this node running.
First add the file 90-hidraw-permissions.rules to the directory
/etc/udev/rules.d/ :
```
sudo cp 90-hidraw-permissions.rules /etc/udev/rules.d/
```

Then add the user to the plugdev group to grant access non-root
to hidraw devices on the system.

```
sudo usermod -aG plugdev $USER
```

Now things should be able to run...

## publishers

| type                     | description                       |
|--------------------------|-----------------------------------|
| [std_msgs/Int32][13]     | linear actuator velocity          |
| [std_msgs/Float64][12]   | drum velocity (radians/sec)       |
| [geometry_msgs/Twist][4] | robot linear and angular velocity |

## parameters

| name         | description                                                                   | type   |
|--------------|-------------------------------------------------------------------------------|--------|
| vel_out      | topic to publish movement velocity on                                         | string |
| arm_out      | topic to publish arm velocity on                                              | string |
| drum_out     | topic to publish drum velocity on                                             | string |
| max_vel      | maximum speed the robot can run at                                            | float  |
| publish_rate | interval in seconds that messages are published on                            | float  |
| device       | serial device the controller is connected on. This is unpredictable           | string |
| interactive  | the node will ask for the serial device and max speed on startup. Recommended | bool   |


## transforms

not relevant

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
[12]: https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/Float64.msg
[13]: https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/Int32.msg
