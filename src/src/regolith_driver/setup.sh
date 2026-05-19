#!/bin/sh

# one-time setup for the regolith_driver node:
#  - installs the teensy udev rules so the regolith controller shows up
#    as /dev/teensy-regolith (avoids /dev/ttyACM* collisions with the
#    roboclaw and other CDC devices)
#  - adds the current user to the dialout group so they can open
#    /dev/ttyACM* without sudo
#
# after running, unplug and replug the teensy. you may also need to
# log out and back in for the dialout group change to take effect.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "getting sudo..."
sudo sleep 1
echo "sudo acquired."

echo "copying udev rules..."
sudo cp "$SCRIPT_DIR/49-teensy.rules" /etc/udev/rules.d/
echo "udev rules copied."

echo "reloading udevadm..."
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "udevadm reloaded."

echo "adding user to dialout group..."
sudo usermod -aG dialout "$USER"
echo "user added to dialout."

echo ""
echo "done. unplug and replug the regolith teensy, then look for"
echo "  /dev/teensy-regolith"
echo "you may need to log out and back in for the dialout group to apply."
