
# src

Ros2 packages used in the system.
Below will be a lengthy explaination of how
the entire stack is supposed to function.    
     
Setup will be listed first, then how to run
things, followed by a description of each node,
how it connects to everything, and what parameters
need to be configured. You may need to edit the
parameters to get some nodes to work.

## setup

### dependencies

The following external libraries need to be installed
in order for the software to function:
    
 * ros2
 * eigen3
 * steam (for x86 machines)
    
ros2 can be installed using the install script. Eigen3
should be in your package manager, and steam you will
have to look up.

### per-node setup

The following nodes have additional setup scripts that need
to be ran in order for them to function:    
    
 * vive_tracker
 * ydlidar_ros2_driver
 * f710

### vive tracker setup

The vive is especially fickle and requires quite a 
bit of extra setup to get running. First, steamvr and
linux-runtime-sniper must be installed from steam. There
are additional instructions in the ``README.md`` for the
node that must be followed to configure steamvr to run
without a headset. Additionally, due to security reasons,
the following command must be ran every time the system
starts before the vive will be able to work:

```
sudo sysctl -w kernel.apparmor_restrict_unprivileged_userns=0
```

Ensure that all of these steps have been completed.
Otherwise the system will not work and you will be confused.

## building

Because many of the computers we are using are constrained
in terms of memory, we must execute colcon build specially
to both allow for nodes that are not being used to fail without
the entire build ending and for the builder to only run a 
single job at a time. The following command should be ran for
building:

```
colcon build --executor sequential --continue
```

## running 

There are two ways to run this system, autonomous and manual.
Each of these will be described below.

### running (manual)

This is a simple mode that requires at a minimum only the
``go_m8010_6``, ``drum_driver``, ``roboclaw_driver``, and
``f710`` nodes to be running. Any additional sensor nodes
can be ran as well if data collection is desired.    
     
If all devices are plugged in on the same machine (reccommended)
the manual control launch script can be ran. Note that this still
expects the controller node to be ran seperately.

```
ros2 launch manual manual.launch.py &
ros2 run f710 f710
```

Otherwise, each node must be ran individually:

```
ros2 run go_m8010_6 go_m8010_6_driver &
ros2 run drum_driver drum_driver &
ros2 run roboclaw_driver roboclaw_driver &
ros2 run f710 f710
```

### running (autonomous)

The autonomous system expects two raspberry pis and
one x86 computer to all be connected on the same
network. Additionally, the following physical connections
are expected:

#### pi 1

 * ydlidar gs2
 * ydlidar gs2
 * ydlidar gs5
 * ydlidar gs5

#### pi 2
 
 * roboclaw motor driver
 * GO M8010-6 motor conenctions

#### x86

 * synexens CS20 lidar
 * vive tracker
    
Provided that this is done, all that should need
to be done is the running of the various launch
files on each machine:

```
# pi 1
ros2 launch pi_1 pi1.launch.py

# pi 2
ros2 launch pi_2 pi2.launch.py

# x86
ros2 launch x86 x86.launch.py
```

In the event that something fails, there should be
a semi-descriptive error message. The error is most
likely a parameter misconfiguration, so double-check
each param file used by the launch scripts or the
parameters set on the nodes themselves. 

## nodes

A description of every node its topics, and its parameters.

### altair

One of the three stars of the summer triad. Path
smoothing node responsible for taking in a raw
path, smoothing it, and outputting the result to
the path follower.    
    
This algorithm internally using the elastic band
algorithm for path smoothing. This algorithm can
be controlled through the parameters given in the
config.

#### subscriptions

| type                           | description                |
|--------------------------------|----------------------------|
| [nav_msgs/Odometry][1]         | recieve the robot position |
| [geometry_msgs/PoseStamped][8] | recieve the goal           |
| [nav_msgs/Path][9]             | recieve the path to smooth |
| [nav_msgs/OccupancyGrid][5]    | recieve the map            |
| [std_msgs/Empty][10]           | reset map topic            |

#### publishers

| type                      | description          |
|---------------------------|----------------------|
| [nav_msgs/Path][9]        | output smoothed path |
| [nav_msgs/MarkerArray][6] | output visualization |

#### parameters

| name                  | description                                                               | type   |
|-----------------------|---------------------------------------------------------------------------|--------|
| map_in                | incoming map topic                                                        | string |
| pos_in                | incoming robot position topic                                             | string |
| pos_in                | incoming goal position topic                                              | string |
| pos_in                | topic path to smooth is being published on                                | string |
| pos_in                | topic reset trigger is tied to                                            | string |
| path_out              | controls the topic the smoothed path is published on                      | string |
| publish_visual        | should the pathing visualization be published?                            | bool   |
| visual_out            | topic to publish the visualization on                                     | string |
| path_publish_interval | time in seconds between path publishing                                   | float  |
| internal_multiplier   | internal force multiplier for the elastic band                            | float  |
| external_multiplier   | external force multiplier for the elastic band                            | float  |
| constraint_multiplier | constraint force multiplier for the elastic band                          | float  |
| distance_cutoff       | maximum distance from an obstace at which the external for is applied     | float  |
| map_height            | score of a map square needed to consider it an obstacle                   | float  |
| bin_width             | unused parameter                                                          | float  |
| max_length            | length of the path used for smoothing                                     | float  |
| hit_distance          | distance a node has to be from an obstacle in order for the path to break | float  |

#### transforms

Expects everything in the map frame.

### bender_5_urdf

This package contains a launch file for
publishing the static transforms of the
robot from a URDF file.

#### transforms

The following transforms are published and
used by various other nodes.

 * base_link
 * rear_gs2_l_link
 * rear_gs2_r_link
 * front_gs5_l_link
 * front_gs5_r_link
 * vectornav_link
 * synexens_link
 * track_l_link
 * track_r_link

The names should be self-explainatory.

### deneb

One of the three stars of the summer triad. Path
following node responsible for taking in a smoothed
path and following it.
      
This algorithm works by computing the rotation error,
then executing an angular velocity if the error is above
a certain threshold. Otherwise a forward linear velocity
is executed. This has the effect that the robot alternates
bewteen rotating and driving forward as it approaches the
next node in the path. The other algorithms should update
the path as the robot moves towards it, so there should
always be more path to follow.

#### subscriptions

| type                           | description                |
|--------------------------------|----------------------------|
| [nav_msgs/Path][9]             | recieve the path to follow |
| [nav_msgs/Odometry][1]         | recieve the robot position |

#### publishers

| type                      | description             |
|---------------------------|-------------------------|
| [geometry_msgs/Twist][9]  | output command velocity |
| [nav_msgs/MarkerArray][6] | output visualization    |

#### parameters

| name                         | description                                                               | type   |
|------------------------------|---------------------------------------------------------------------------|--------|
| path_in                      | incoming path to follow                                                   | string |
| pos_in                       | incoming position topic                                                   | string |
| vel_out                      | velocity topic to publish output on                                       | string |
| publish_visual               | whether visual information should be published                            | bool   |
| visual_out                   | topic to publish the visualization on                                     | string |
| vel_publish_interval         | time in between velocity publish commands                                 | float  |
| angular_correction_threshold | angular error (radians) at which a rotation is executed                   | float  |
| hit_distance                 | distance from the next path node we consider it 'reached'                 | float  |
| angular_velocity_scale       | speed (radians/sec) the robot will turn                                   | float  |
| linear_velocity_scale        | speed (meters/sec) the robot will drive forward                           | float  |

#### transforms

Expects everything in the map frame.

### drum_driver

simple node wrapping the unitree go motor sdk for
spinning the regolith drum.

#### subscriptions

| type                   | description                 |
|------------------------|-----------------------------|
| [std_msgs/Float64][12] | drum velocity (radians/sec) |

#### parameters

| name             | description                                             | type   |
|------------------|---------------------------------------------------------|--------|
| motor_id         | internal id the motor has been set to                   | int    |
| position_damping | multiplier on the positional error                      | float  |
| velocity_damping | multiplier on the velocity error                        | float  |
| cmd_id           | topic to recieve velocity commands on                   | string |
| motor            | serial device the motor is. Normally a ``/dev/ttyUSBX`` | string |

#### transforms

not relevant

### ekf

unused and unfinished node

### f710

used for manually driving the robot.

#### publishers

| type                     | description                       |
|--------------------------|-----------------------------------|
| [std_msgs/Int32][13]     | linear actuator velocity          |
| [std_msgs/Float64][12]   | drum velocity (radians/sec)       |
| [geometry_msgs/Twist][4] | robot linear and angular velocity |

#### parameters

| name         | description                                                                   | type   |
|--------------|-------------------------------------------------------------------------------|--------|
| vel_out      | topic to publish movement velocity on                                         | string |
| arm_out      | topic to publish arm velocity on                                              | string |
| drum_out     | topic to publish drum velocity on                                             | string |
| max_vel      | maximum speed the robot can run at                                            | float  |
| publish_rate | interval in seconds that messages are published on                            | float  |
| device       | serial device the controller is connected on. This is unpredictable           | string |
| interactive  | the node will ask for the serial device and max speed on startup. Recommended | bool   |


#### transforms

not relevant

### go_m8010_6

differential drive controller for the robot tracks.

## subscriptions

| type                     | description                       |
|--------------------------|-----------------------------------|
| [geometry_msgs/Twist][4] | robot linear and angular velocity |

## parameters

| name             | description                                                  | type   |
|------------------|--------------------------------------------------------------|--------|
| cmd_in           | topic to recieve robot linear and angular velocity on        | string |
| left_motor       | serial device of the left motor. Usually a ``/dev/ttyUSBX``  | string |
| right_motor      | serial device of the right motor. Usually a ``/dev/ttyUSBX`` | string |
| left_motor_id    | internal set id of the left unitree motor                    | int    |
| right_motor_id   | internal set id of the right unitree motor                   | int    |
| position_damping | error multiplier for position in the motor PD controllers    | float  |
| velocity_damping | error multiplier for velocity in the motor PD controllers    | float  |

## transforms

not relevant

### manual

Manual control launch script for the robot. This will launch
the roboclaw_driver, drum_driver, and go_m8010_6 nodes using
the launch file given in the package.

### pc_flatten

filter for all lidar sensors on the robot. Has a bunch of configurable
settings and stuff.

#### usage

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

#### subscriptions

| type                           | description            |
|--------------------------------|------------------------|
| [geometry_msgs/PointCloud2][3] | point cloud to process |
| [geometry_msgs/LaserScan][2]   | laser scan to process  |

#### publishers

| type                           | description                         |
|--------------------------------|-------------------------------------|
| [geometry_msgs/PointCloud2][3] | final filtered cloud output         |
| [geometry_msgs/PointCloud2][3] | z range culled points in debug mode |
| [geometry_msgs/PointCloud2][3] | lowpass culled points in debug mode |
| [geometry_msgs/PointCloud2][3] | box culled points in debug mode     |

#### parameters

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

#### transforms

This node expects to be able to transform from the frame it recieves sensors
on to the frame that is set in the parameter frame_out. This transform should
be doable. Additionally, the frames of inputs are used to flag the lowpass and
box filters.

### pi_1

Launch file for the pi connected to each gs2/gs5 lidar. This launch file
launches only these nodes and I am pretty sure nothing else.

### pi_2

Launch file for the pi connected to the motors and linear actuators.
This node is also responsible for publishing the transforms.

### roboclaw_driver

Port of the roboclaw controller library that
works on unix. Accepts a velocity input command
and (in our system) drives a linear actuator.

#### subscriptions

| type                 | description                                       |
|----------------------|---------------------------------------------------|
| [std_msgs/Int32][13] | velocity (arbitrary units) to run the actuator at |

#### parameters

| name         | description                                                              | type   |
|--------------|--------------------------------------------------------------------------|--------|
| robclaw_port | serial device the roboclaw connects on. Usually through ``/dev/ttyACMX`` | string |
| baud         | baud rate for the controller communication                               | int    |
| cmd_vel_in   | topic to recieve our velocity commands on                                | string |
| address      | internal set address of the roboclaw controller                          | int    |

#### transforms

does not apply

### simple_map

Grid map the robot builds when navigating. Works
by adding the laser scan or point cloud messages
it recieves onto the map and updating the weights
of every square that an obstacle touches. When these
squares exceed a threshold they are treated as obstacles.

#### subscriptions

| type                         | description                           |
|------------------------------|---------------------------------------|
| [nav_msgs/Odometry][1]       | accepts current position of the robot |
| [sensor_msgs/LaserScan][2]   | accepts sensor input                  |
| [sensor_msgs/PointCloud2][3] | accepts point cloud input             |
| [std_msgs/Empty][10]         | accepts the reset trigger             |

#### publishers

| type                        | description     |
|-----------------------------|-----------------|
| [nav_msgs/OccupancyGrid][5] | map being built |

#### parameters

| name                   | description                                                          | type   |
|------------------------|----------------------------------------------------------------------|--------|
| map_out                | topic to publish the map on                                          | string |
| map_publish_interval   | time in seconds between map publishing                               | float  |
| publish_map_frame      | should the map -> odom transform be published                        | bool   |
| frame_publish_interval | time in seconds between map -> odom transform publish                | float  |
| pos_in                 | topic to recieve odometry on                                         | string |
| points_in              | topic to recieve point cloud data on                                 | string |
| scan_in                | topic to recieve laser scan data on                                  | string |
| reset_in               | topic to recieve reset trigger messages on                           | string |
| map_resolution         | size of a grid square in meters                                      | float  |
| map_hit_weight         | value added to a grid square when a obstacle is seen in it           | int    |
| map_miss_weight        | value removed from a grid square when a lidar scan passes through it | int    |
| map_start_weight       | starting value in the grid squares (note squares cap at 100)         | int    |

#### transforms

This node expects to be able to transform any incoming sensor data into the odom frame. It
additionally publishes the map -> odom transform as a static 0. Expects odometry in the odom
or map frame.

### synexens

Driver node for the synexens CS20 lidar. External, see their docs for details.

### test_env

testing environment. Unused here

### tf_tune

program for tuning transforms between two
reference frames. Shouldn't be needed here.

### vega

One of the three stars in the summer triad. This is the path generator node.
It is implimented as an A* algorithm with a few modifications. Pathing is
done from the start to the goal. Each obstacle has a radius around it expanded
to keep the robot farther away. Repathing is done on a short interval periodically,
and full repathing is done over a longer interval.

#### subscriptions

| type                           | description                           |
|--------------------------------|---------------------------------------|
| [nav_msgs/OccupancyGrid][5]    | map to path over                      |
| [nav_msgs/Odometry][1]         | accepts current position of the robot |
| [geometry_msgs/PoseStamped][2] | accepts the goal position             |
| [std_msgs/Empty][10]           | accepts the reset trigger             |

#### publishers

| type                            | description                              |
|---------------------------------|------------------------------------------|
| [nav_msgs/OccupancyGrid][5]     | map after post-processing in visual mode |
| [nav_msgs/Path][9]              | path generated                           |
| [visualization_msgs/Marker][11] | visualization of the algorithm           |

#### parameters

| name                   | description                                                                | type   |
|------------------------|----------------------------------------------------------------------------|--------|
| map_in                 | topic to recieve the map on                                                | string |
| pos_in                 | topic to recieve the robot position on                                     | string |
| goal_in                | topic to recieve the goal on                                               | string |
| reset_in               | topic to recieve the reset callback on                                     | string |
| path_out               | topic to publish the path on                                               | string |
| path_publish_interval  | time interval in seconds the path is published on                          | float  |
| visual_out             | topic to publish visualization on                                          | string |
| map_out                | topic to publish the post-processed map on                                 | string |
| map_obstacle_threshold | minimum distance (in meters) the path can be from an obstacle              | float  |
| publish_visual         | whether the visualization should be published                              | bool   |
| map_resolution         | width of the map grid squares in meters                                    | float  |
| map_hit_weight         | threshold at which a map square is considered an obstacle                  | int    |
| map_miss_weight        | arbitrary parameter                                                        | int    |
| map_start_weight       | arbitrary parameter                                                        | int    | 
| repath_range           | distance (in meters) in front of the robot that is repathed every interval | float  |
| full_repath_interval   | time (in seconds) between full repaths                                     | float  |

#### transforms

everything in the map frame

### vive_to_odom

Takes in vive tracker data, publishes the odom -> base_link transform
and the odom data from the base_link reference frame.

#### subscriptions

| type                   | description                                                |
|------------------------|------------------------------------------------------------|
| [nav_msgs/Odometry][1] | odometry from a reference frame transformable to base_link |

#### publishers

| type                   | description                          |
|------------------------|--------------------------------------|
| [nav_msgs/Odometry][1] | odometry in the odom reference frame |

#### parameters

| name                   | description                                                            | type   |
|------------------------|------------------------------------------------------------------------|--------|
| frame_publish_interval | time interval in seconds to publish the odom -> base_link transform on | float  |
| pos_in                 | topic to recieve odometry messages on                                  | string |
| pos_out                | topic to publish transformed odometry on                               | string |

#### transforms

This node is responsible for publishing odom -> base_link. This node
expects to be able to transform the incoming odometry frame id to base_link.

### vive_tracker

Driver node for the vive tracker. External. Check the docs on this node for details.

### waveshare_driver

Differential drive controller for the waveshare rover.

### waveshare_urdf

URDF for the waveshare rover.

### x86

Launch file for the x86 computer. Starts the vive tracker and
the synexens. Also the pc_flatten

### ydlidar_ros2_driver

Driver node for the ydlidar gs2 and gs5. External, see their docs for details.

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
