
# Central Controller Node

## Overview

The node implements a simple state machine with the following states:
- Search for dig zone
- Navigate to dig zone  
- Dig  
- Search for dump zone
- Navigate to dump zone  
- Dump  

Basically, it takes in the robot's position and the generated map and waits for the dig and dump zone perception nodes to publish the locations of the dig and dump zones. Then,it publishes the goal positions to the path generator, the arm angle and drum speed to the regolith system, and then runs a state-machine to determine the robot's next course of action. This is the general decision-maker of the robot.

