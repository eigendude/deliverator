#!/bin/bash

# Installing ROS Melodic on the Raspberry Pi
# Based on Kinetic instructions
# http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi

ROS_DISTRO=melodic
DEBIAN_VERSION=stretch

# Choose the particular variant you want to install:
#   - ros_comm: ROS package, build, and communication libraries. No GUI tools.
#   - desktop: ROS, rqt, rviz, and robot-generic libraries
#   - desktop_full: ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators,
#                   navigation and 2D/3D perception
ROS_VARIANT=desktop_full

CATKIN_WORKSPACE=$HOME/catkin_ws
INSTALL_LOCATION=/opt/ros/$ROS_DISTRO

# Setup ROS Repositories
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update

# Install bootstrap dependencies
sudo apt-get install -y python-rosdep python-rosinstall-generator \
    python-wstool python-rosinstall build-essential cmake

# Initialize rosdep
sudo rosdep init
rosdep update

# Create a catkin Workspace
mkdir -p $CATKIN_WORKSPACE
cd $CATKIN_WORKSPACE

ROS_INSTALL_FILE=$ROS_DISTRO-$ROS_VARIANT-wet.rosinstall
rosinstall_generator $ROS_VARIANT --rosdistro $ROS_DISTRO --deps --wet-only --tar > $ROS_INSTALL_FILE
wstool init -j8 $CATKIN_WORKSPACE/src $ROS_INSTALL_FILE

# If wstool init fails or is interrupted, you can resume the download by running:
#wstool update -j 4 -t $CATKIN_WORKSPACE/src

# Dependencies not available in the Raspbian stable branch
mkdir -p $CATKIN_WORKSPACE/external_src
cd $CATKIN_WORKSPACE/external_src

echo > /dev/null << EOF
if [ "$ROS_VARIANT" == "desktop" ]; then
  # Building the collada_urdf package fails to link to Assimp
  # http://answers.ros.org/question/48084/urdf_to_collada-undefined-reference-to-vtable-for-assimpiosystem/
  if [ ! -d assimp-3.1.1 ]; then
    wget --continue http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
    unzip assimp-3.1.1_no_test_models.zip
  fi
  cd assimp-3.1.1
  cmake .
  make -j4
  sudo make install
  cd ..
fi
EOF

# Resolving dependencies with rosdep
cd $CATKIN_WORKSPACE
rosdep install --from-paths $CATKIN_WORKSPACE/src --ignore-src --rosdistro $ROS_DISTRO -y -r --os=debian:$DEBIAN_VERSION

# Building the catkin Workspace
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space $INSTALL_LOCATION

# Add the following to .bashrc
source /opt/ros/melodic/setup.bash
