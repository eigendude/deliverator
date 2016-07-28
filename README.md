# Deliverator

![Cosa Nostra Pizza Delivery](Cosa_Notra_Pizza_Delivery_Vehicle_by_Igor_Sobolevsky.jpg "The Deliverator")

Image credit: https://www.artstation.com/artwork/cosa-nostra-pizza-delivery-vehicle

## Setup Instructions

### Contents
1. [Set up Raspberry Pis with ROS](#1-set-up-raspberry-pis-with-ros)
2. [Create catkin workspace](#2-create-catkin-workspace)
3. [Clone repository](#3-clone-repository)
4. [Configure which packages to use](#4-configure-which-packages-to-use)
5. [Download dependencies](#5-download-dependencies)
6. [Compilation](#6-compilation)
7. [Test Setup in Simulation](#7-test-setup-in-simulation)

### 1. Set up Raspberry Pis with ROS

If installing on a Raspberry Pi, see [README-PI.md](README-PI.md) for instructions on installing Rasbian and compiling ROS from source.

### 2. Create catkin workspace

If you aren't cloning to an existing [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), you should create one for this project now:

```shell
mkdir -p ~/deliverator_ws/src
cd ~/deliverator_ws/src
catkin_init_workspace
cd ..
catkin_make
```

Before continuing source your new setup.*sh file (and add to `.bashrc`):

```shell
source ~/deliverator_ws/devel/setup.bash
```

### 3. Clone repository

Clone the deliverator repository into your catkin workspace.

```shell
cd ~/deliverator_ws/src
git clone https://github.com/juztamau5/deliverator.git
```

### 4. Configure which packages to use

Lightweight nodes (such as the RPi Zero) can disable packages to avoid heavy dependencies (such as Gazebo).

Packages can be whitelisted or blacklisted. Whitelisting can be accomplished using git sparse checkout:

```shell
cd deliverator
git config core.sparsecheckout true
echo gazebo_ros_core/ > .git/info/sparse-checkout
git checkout
```

Packages can also be blacklisted by adding a `CATKIN_IGNORE` file to their folder:

```shell
touch deliverator_gazebo/CATKIN_IGNORE
```

### 5. Download dependencies

Dependencies are installed using `rosdep`.

```shell
rosdep install --from-path . --ignore-src -y
```

If `rosdep` can't locate a package for the system, it will fail and report the package name. If a package is missing for your system, match the error message below and follow the commands.

#### Cannot locate rosdep definition for [gscam]

```shell
git clone https://github.com/ros-drivers/gscam.git
```

#### Missing resource camera_calibration_parsers

```shell
git clone https://github.com/ros-perception/image_common.git
```

#### Unable to locate package ros-kinetic-ackermann-msgs

```shell
git clone https://github.com/ros-drivers/ackermann_msgs.git
```

#### Unable to locate package ros-kinetic-joy

```shell
git clone https://github.com/ros-drivers/joystick_drivers.git
```

#### Missing gazebo_ros_control

```shell
git clone -b kinetic-devel https://github.com/ros-simulation/gazebo_ros_pkgs.git
cd gazebo_ros_pkgs
git config core.sparsecheckout true
echo gazebo_ros_control/ > .git/info/sparse-checkout
git checkout
cd ..
```

#### Missing hector_gazebo

```shell
git clone -b kinetic-devel https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git
cd hector_gazebo
git config core.sparsecheckout true
echo hector_gazebo_plugins/ > .git/info/sparse-checkout
git checkout
cd ..
```

#### fatal error: netlink/genl/ctrl.h: No such file or directory

```shell
sudo apt-get install libnl-3-dev libnl-genl-3-dev
```

#### Could not find a package configuration file provided by "libp8_platform"

```shell
git clone --recursive https://github.com/juztamau5/libp8_platform.git
```

### 6. Compilation

To compile run `catkin_make` from the catkin workspace folder.

```shell
cd ~/deliverator_ws
catkin_make
```

### 7. Test Setup in Simulation

To test that your setup process was successful, run the simulator with the following command:

```shell
roslaunch deliverator_gazebo ackermann_vehicle.launch
```
