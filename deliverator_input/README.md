# Deliverator Input

This package contains drivers for input devices such as joysticks.

This has been tested with a wireless Xbox 360 controller with a cheap knockoff receiver.

## Setup Instructions

This package can be run on nodes with little processing power, such as the Raspberry Pi Zero. The setup instructions assume that the catkin workspace is set up like the top-level [README.md](../README.md).

## Dependencies

This package depends on the ROS joy driver, which is not packaged for debian yet. Clone it manually:

```shell
cd ~/deliverator_ws/src
git clone https://github.com/ros-drivers/joystick_drivers.git
```

Once this dependency is in place, the system depends can be installed with rosdep:

```shell
rosdep install -y --from-paths . --ignore-src --rosdistro kinetic -r --os=debian:jessie
```

## Compilation

To compile run `catkin_make` from the catkin workspace folder.

```shell
cd ~/deliverator_ws
catkin_make
```
