# Deliverator

![Cosa Nostra Pizza Delivery](Cosa_Notra_Pizza_Delivery_Vehicle_by_Igor_Sobolevsky.jpg "The Deliverator")

Image credit: https://www.artstation.com/artwork/cosa-nostra-pizza-delivery-vehicle

## Setup Instructions

### Contents
1. [Create catkin workspace](#1-create-catkin-workspace)
2. [Download dependencies](#2-download-dependencies)
3. [Clone repository](#3-clone-repository)
4. [Install ROS Dependencies](#4-install-ros-dependencies)
5. [Compilation](#5-compilation)
6. [Test Setup in Simulation](#6-test-setup-in-simulation)
7. [Set up Raspberry Pi](#7-set-up-raspberry-pi)

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
rosdep install --from-path . --ignore-src -y
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

### 7. Set up Raspberry Pi

The Raspberry Pis in this project use Raspbian based on Debian Jessie. If you're using Raspbian, these tips might come in handy.

#### 7.1 Rename default user

Log in to the Raspberry Pi (password is "raspberry"):

```shell
ssh pi@192.168.1.2
```

Create temp user:

```shell
sudo useradd --create-home --gid sudo --shell /bin/bash temp
sudo passwd temp
```

Log out, then log in as temp:

```shell
exit
ssh temp@192.168.1.2
```

Rename default "pi" user:

```shell
sudo usermod --login <username> pi
sudo usermod --home /home/<username> --move-home <username>
```

Optional, rename the group:

```shell
sudo groupmod --new-name <groupname> pi
```

Log out, then log in with new username (password is still "raspberry"):

```shell
exit
ssh <username>@192.168.1.2
```

Don't forget to change your password and remove the temporary user:

```shell
passwd
sudo userdel --remove temp
```

Finally, erase the line `pi ALL=(ALL) NOPASSWD: ALL` from the sudoers file using visudo:

```shell
sudo visudo
```

#### 7.2 Set timezone

First, check the current timezone:

```shell
date +'%:z %Z'
```

To change the timezone, use:

```shell
sudo dpkg-reconfigure tzdata
```

#### 7.3 Install ROS from source

First, check if ROS can be installed from the package manager. This will save you a lot of time and space.

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
apt-cache search ros-kinetic-*
```

If `apt-cache search` returns results, you can follow the normal [Ubuntu installation steps](http://wiki.ros.org/kinetic/Installation/Ubuntu). Otherwise, you'll have to install from source.

Installing from source takes about 8.5 hours (when run by a script) and uses 3.7 GB above a vanilla Debian Jessie install (1 GB). 4.7 GB total means you'll need at least a 8 GB microSD card.

Before running the script, you should ensure that your **swap is big enough** and that your **sudo timeout is long enough**.

##### 7.3.1 Increasing swap

Raspbian ships with a 100MB swap file. Compiling ROS with `-j4` on the Raspberry Pi 2 exceeds this limited space.

1GB of swap has been observed to be deficient. I had success with 8GB, the sweet spot is probably somewhere in the middle.

```shell
sudo swapoff --all
sudo dd if=/dev/zero of=/var/swap bs=1M count=8096
sudo mkswap /var/swap
sudo swapon /var/swap
```

This will enlarge the swap file to 8GB. Swapping to the microSD is slow, so you may want to put the swap file on external storage. I connected a SSD to a spare SATA-to-USB3 adapter and got 260 Mbps read/write over the RPi's USB controller.

##### 7.3.2 Extending sudo timeout

The script takes several hours to complete, so to avoid interruptions it is recommended that you disable sudo timeouts until the script is finished:

```shell
sudo visudo
```

Add the following line to disabled timeouts:

```
Defaults  timestamp_timeout=-1
```

After the script is complete, you can set the timeout to something more reasonable, like 60 minutes:

```
Defaults  timestamp_timeout=60
```

##### 7.3.3 Performing the install

A script has been added to the `deliverator_util` package to perform the entire install. It follows the instructions on the [Installing ROS Kinetic on the Raspberry Pi](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi) documentation article, so if the script breaks the offending line will be explained in the article.

Before downloading and blindly executing a script from the internet, view the source here: [raspbian-install-ros-kinetic.sh](https://github.com/juztamau5/deliverator/blob/master/deliverator_util/raspbian-install-ros-kinetic.sh). Then download and run:

```shell
wget https://raw.github.com/juztamau5/deliverator/master/deliverator_util/raspbian-install-ros-kinetic.sh
chmod +x raspbian-install-ros-kinetic.sh
time ./raspbian-install-ros-kinetic.sh
```

As mentioned before, this takes around 8.5 hours on the Raspberry Pi 2 and generates 3.7 GB worth of files. It really sucks when this script breaks and has to be restarted, so if you run into a problem please open an [issue](https://github.com/juztamau5/deliverator/issues) so I can update the script and the wiki.
