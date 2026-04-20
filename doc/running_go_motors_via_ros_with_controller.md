
# Go motors via f710

This will be listed in stages.

First get a f710 controller and plug the usb dongle
into the computer. This will show up as /dev/hidrawX
where X is a number. There are normally several hidraws
so you should determine which is the f710. This can
be done by catting the values and seeing which one
prints when you move the controller.

next ensure that the setup in the src/src/f710/README.md
has been completed.

next run the f710 node:
```
cd src
colcon build
source install/setup.sh
ros2 run f710 f710
```
you should be prompted for the port. Enter the right
one you determined earlier. Now you will be prompted for
speed. Enter the appropriate value.

now open a new terminal. Navigate to the src directory
and run the go motor driver program.
```
cd src
colcon build
source install/setup.sh
ros2 run go_m8010_6 go_m8010_6_driver
```

This will print out the ttyUSB objects it is connected
to and the motor ids. If this does not work you should
ensure that the motors 0 and 1 are connected to the usb
values it is listing. You might have to switch the ports
on the connecter or swap them until it works. You can
always try plugging one in and moving it along until
you get output, then doing the same with the other.

Things should work now. Enjoy!
