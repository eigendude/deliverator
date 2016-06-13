# Deliverator

![Cosa Nostra Pizza Delivery](https://raw.githubusercontent.com/juztamau5/deliverator/master/Cosa_Notra_Pizza_Delivery_Vehicle_by_Igor_Sobolevsky.jpg "The Deliverator")

Image credit: https://www.artstation.com/artwork/cosa-nostra-pizza-delivery-vehicle

## Setup Instructions

### Contents
1. [Install Prerequisites](#1-install-prerequisites)
2. [Clone repository](#2-clone-repository)
3. [Compilation/Running](#3-compilationrunning)
4. [Generate Documentation](#4-generate-documentation)
5. [Test Setup in Simulation](#5-test-setup-in-simulation)

### 1. Install Prerequisites

The Deliverator package depends on gscam for camera usage. If gscam isn't available for your current ROS version, enter your [catkin workspace](http://wiki.ros.org/catkin/workspaces) and clone it manually:

```shell
cd src
git clone https://github.com/ros-drivers/gscam.git
```

gscam doesn't fully install all dependencies. gscam may fail at runtime if the following package isn't installed:

```shell
sudo apt-get install gstreamer0.10-plugins-good
```

The Deliverator package also depends on gazebo_ros_control, which may not be available for your current ROS version. Per [this answer](http://answers.ros.org/question/235846/did-ros-kinetic-gazebo-ros-control-package-release/), it can be cloned as follows:

```shell
git clone -b kinetic-devel https://github.com/ros-simulation/gazebo_ros_pkgs.git
cd gazebo_ros_pkgs
git config core.sparsecheckout true
echo gazebo_ros_control/ >> .git/info/sparse-checkout
git checkout
cd ..
```

### 2. Clone Repository

Clone the deliverator repository into your catkin workspace. The suggested location is `~/catkin_ws/src/`, but any valid catkin worskspace source folder will work.

```shell
git clone https://github.com/juztamau5/deliverator.git
```

### 3. Install ROS Dependencies

If the prerequisites above are avaiable, installing dependencies from the catkin workspace folder should now work:

```shell
cd ../..
rosdep update --from-path ./src --ignore-src -y
```

### 3. Compilation & Running

To compile run `catkin_make` from the catkin workspace folder.

_Note:_ If you are unfamiliar with catkin, please know that you must run `source <catkin_ws>/devel/setup.sh` before ROS will be able to locate the deliverator packages. This line can be added to your ~/.bashrc file.

### 5. Test Setup in Simulation

To test that your setup process was successful, run the simulator with the following command:

```shell
roslaunch deliverator_gazebo ackermann_vehicle.launch
```

