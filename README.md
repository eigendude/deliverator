# Deliverator

![Cosa Nostra Pizza Delivery](https://cdn2.artstation.com/p/assets/images/images/001/167/838/large/igor-sobolevsky-dtf-cn-front-hero-copy.jpg?1443928195&dl=1 "The Deliverator")

Image credit: https://www.artstation.com/artwork/cosa-nostra-pizza-delivery-vehicle

## Setup Instructions

### Contents
1. [Install Prerequisites](#1-install-prerequisites)
2. [Clone repository](#2-clone-repository)
3. [Compilation/Running](#3-compilationrunning)
4. [Generate Documentation](#4-generate-documentation)
5. [Test Setup in Simulation](#5-test-setup-in-simulation)

### 1. Install Prerequisites

The Deliverator package depends on gscam for camera usage. If gscam isn't available for your current ROS version, clone it to your [catkin workspace](http://wiki.ros.org/catkin/workspaces) and build it manually:

```shell
cd src
git clone https://github.com/ros-drivers/gscam.git
rosdep update --from-path . --ignore-src
catkin_make
```

gscam doesn't fully install all dependencies. gscam may fail at runtime if the following package isn't installed:

```shell
sudo apt-get install gstreamer0.10-plugins-good
```

### 2. Clone Repositories

Clone the deliverator repository into your catkin workspace. The suggested location is `~/catkin_ws/src/`, but any valid catkin worskspace source folder will work.

```git clone https://github.com/juztamau5/deliverator.git```

### 3. Install ROS Dependencies

Within the catkin workspace folder, run this command to install the packages this project depends on.

```rosdep install --from-path src --ignore-src -y```

### 3. Compilation & Running

To compile and install run `catkin_make` from the catkin workspace folder.

_Note:_ If you are unfamiliar with catkin, please know that you must run `source <catkin_ws>/devel/setup.sh` before ROS will be able to locate the autorally packages. This line can be added to your ~/.bashrc file.

### 5. Test Setup in Simulation

To test that your setup process was successful, run the simulator with the following command:

```roslaunch deliverator_gazebo gazeboSim.launch```
