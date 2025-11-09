
# Differential Drive Controller

## Overview

This node has the job of translating a goal angular
and linear velocity that is provided by the output of
some navigation system into motor speeds for the left
and right tracks.

## Inputs

| name    | msg type                 | description                         |
|---------|--------------------------|-------------------------------------|
| cmd_vel | [geometry_msgs/Twist][1] | Desired linear and angular velocity |

## Outputs

| name        | msg type   | description                                      |
|-------------|------------|--------------------------------------------------|
| Left Motor  | raw output | Command the left motor to move at a given speed  |
| Right Motor | raw output | Command the right motor to move at a given speed |

## Implimentation

The differential drive math will be copied from the 
[wikipedia page](https://en.wikipedia.org/wiki/Differential_wheeled_robot)
describing its implimentation.    
    
Constants for widths and radiuses will be exposed as needed for ease
of use later on.
