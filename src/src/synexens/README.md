# Synexens Ros2 Iron
A small node constructed using the Synexens LiDAR SDK as reference. This was created because I was unable to get 
the Synexens sensor's provided Ros2 nodes on my develop environement - which uses Ros2 Iron. They did not claim
support for Iron though, so it is not a big deal.
## Usage
Building is essentially normal:
```
cd ./ros2_workspace
cd ./src
git clone git@github.com:qeftser/synexens_iron.git
cd ..
colcon build
source install/setup.sh
```
To run the node:
```
ros2 launch synexens_lidar launch.py
```
To run the node + a rviz visualization of the data:
```
rso2 launch synexens_lidar view_launch.py
```
There is a config file that the node uses to set itself up. Documentation for the 
various avaliabe parameters are avaliable in the file config/synexens.yaml.
## Topics
There is a single topic published, which is a sensor_msgs/PointCloud2 message. The topic and frame_id it is published
on is configurable. There are some Synexens LiDAR that will output RGB and other data, but I did not have any of 
those on hand, so those features did not get implimented. So depth is all that is supported currently.
