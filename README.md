# Deliverator

![Cosa Nostra Pizza Delivery](https://raw.githubusercontent.com/juztamau5/deliverator/master/Cosa_Notra_Pizza_Delivery_Vehicle_by_Igor_Sobolevsky.jpg "The Deliverator")

Image credit: https://www.artstation.com/artwork/cosa-nostra-pizza-delivery-vehicle

## Setup Instructions

### Contents
1. [Create catkin workspace](#1-create-catkin-workspace)
2. [Download dependencies](#2-download-dependencies)
3. [Clone repository](#3-clone-repository)
4. [Install ROS Dependencies](#4-install-ros-dependencies)
5. [Compilation](#5-compilation)
6. [Test Setup in Simulation](#6-test-setup-in-simulation)

### 1. Create catkin workspace

If you aren't cloning to an existing [catkin workspace](http://wiki.ros.org/catkin/workspaces), you should create one for this project now:

```shell
mkdir -p ~/deliverator_ws/src
cd ~/deliverator_ws/src
catkin_init_workspace
cd ..
catkin_make
```

Before continuing source your new setup.*sh file:

```shell
source ~/deliverator_ws/devel/setup.bash
```

### 2. Download dependencies

The Deliverator package depends on gscam for camera usage. If gscam isn't available for your current ROS version, enter your catkin workspace and clone it manually:

```shell
cd src
git clone https://github.com/ros-drivers/gscam.git
```

The Deliverator package also depends on gazebo_ros_control, which may not be available for your current ROS version. Per [this answer](http://answers.ros.org/question/235846/did-ros-kinetic-gazebo-ros-control-package-release/), it can be cloned as follows:

```shell
git clone -b kinetic-devel https://github.com/ros-simulation/gazebo_ros_pkgs.git
cd gazebo_ros_pkgs
git config core.sparsecheckout true
echo gazebo_ros_control/ > .git/info/sparse-checkout
git checkout
cd ..
```

The Gazebo model uses plugins from [hector_gazebo_plugins](http://wiki.ros.org/hector_gazebo_plugins):

```shell
git clone -b jade-devel https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git
cd hector_gazebo
git config core.sparsecheckout true
echo hector_gazebo_plugins/ > .git/info/sparse-checkout
git checkout
cd ..
```

### 3. Clone Repository

Clone the deliverator repository into your catkin workspace.

```shell
git clone https://github.com/juztamau5/deliverator.git
```

### 4. Install ROS Dependencies

Installing dependencies from the catkin workspace folder should now work:

```shell
rosdep update --from-path . --ignore-src -y
```

gscam may not fully list all dependencies. gscam may fail at runtime if the following package isn't installed:

```shell
sudo apt-get install gstreamer0.10-plugins-good
```

### 5. Compilation

To compile run `catkin_make` from the catkin workspace folder.

_Note:_ If you are unfamiliar with catkin, please know that you must run `source <catkin_ws>/devel/setup.sh` before ROS will be able to locate the deliverator packages. This line can be added to your `~/.bashrc` file.

### 6. Test Setup in Simulation

To test that your setup process was successful, run the simulator with the following command:

```shell
roslaunch deliverator_gazebo ackermann_vehicle.launch
```
