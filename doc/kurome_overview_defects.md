
# Kurome 

This is an explanation of the combination of systems used by the kurome
navigation stack and some of the shortcomings and issues surrounding the
implimentation and choice of algorithms. This is not meant to be a
tutorial on each of the algorithms used, but rather a brief description
of them followed by a commentary on their effectivness in the light of
the competition.

## Path Follower

The path following algorithm used in kurome was a very simple one that
turned to face the next point in the path and then moved towards it. This
was done until the end point was reached.

### Commentary

This approach was constructed after several more mathmatically complex
approaches were tried. It proved effective for our uses, but relied on
the ability of the robot to turn in place and did not take into account
the path outside of the immidiate next node. This lead to some lost 
opportunity for smoothing or taking advantage of the robot's momentum.

## Map Generation

A 2d grid map was used as the internal map representation for kurome. Path
planners would take the base map and draw a circle around each obstacle to
ensure that they stayed a set distance from it. The base map was constructed
using a graphslam algorithm and some simple probability.

### Commentary

This worked well for our uses. The map was kind of big and got copied around
a lot in the system, but a new one was only sent once a second, so there was
no obvious slowdown seen from it. Issues mostly occured with the map generation
and usage :/.

## SLAM

The slam method used in kurome was a graph slam front and back end that incorporated
odometry and beacon input with lidar and point cloud measurements. Ceres was used
on the backend as the nonlinear optimizer. 

### Commentary

The back end worked correctly but the front
end continually (and still does) lacked a method for generating coorespondences between
sensor measurements to use for loop closure. This problem will have to be fixed this year
if we want to use any kind of slam system.

## Pathing Stage 1

A* was used as the first stage of the path planning in kurome. The A* implimentation was
custom in that it could be tuned to only replan a subset of the path every step or only
repath when a new obstacle cut the path off.

### Commentary

The cost function the A* used generated some interesting results at times but the system was
otherwise pretty effective in the overall navigation stack.

## Pathing Stage 2

The elastic band algorithm was used to smooth the A* path. This was the standard implimentation
from the original thesis, not the timed elastic band extension.

### Commentary

This method worked well in most cases, but really struggled when the size of the path got too large.
This is due in part to how the algorithm works but also because it is an optimization thing so is
not very fast. This is something that should probably change going forward. There were also some issues
where A* would generate a path that elastic band did not see as feasable. This would end up blocking
the elastic band until A* produced something different. A method for dealing with this would be nice,
as before we would just tune parameters until the problem went away.

## Sensors

There is not much to say about sensor unfortunately, as we did not get them integrated with the
robot last year. We will have to see going forward what to do.

## Sensor Fusion

The default robot localization node was used last year to fuse odometry and imu input. Not a ton
to say about this other than that it appeared to work well.
