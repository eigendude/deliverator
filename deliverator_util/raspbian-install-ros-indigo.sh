#!/bin/bash

# Installing ROS Indigo on Raspberry Pi
# http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi

ROS_DISTRO=indigo
DEBIAN_VERSION=wheezy

# Choose the particular variant you want to install:
#   - ros_comm: ROS package, build, and communication libraries. No GUI tools.
#   - desktop: ROS, rqt, rviz, and robot-generic libraries
ROS_VARIANT=desktop

CATKIN_WORKSPACE=~/catkin_ws
INSTALL_LOCATION=/opt/ros/$ROS_DISTRO

# Setup ROS Repositories
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu wheezy main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update

# Install bootstrap dependencies
sudo apt-get install -y python-setuptools
sudo easy_install pip
sudo pip install -U rosdep rosinstall_generator wstool rosinstall

# Initialize rosdep
sudo rosdep init
rosdep update

# Create a catkin Workspace
mkdir -p $CATKIN_WORKSPACE
cd $CATKIN_WORKSPACE

ROS_INSTALL_FILE=$ROS_DISTRO-$ROS_VARIANT-wet.rosinstall
rosinstall_generator $ROS_VARIANT --rosdistro $ROS_DISTRO --deps --wet-only --exclude roslisp --tar > $ROS_INSTALL_FILE
wstool init -j8 $CATKIN_WORKSPACE/src $ROS_INSTALL_FILE

# If wstool init fails or is interrupted, you can resume the download by running:
#wstool update -j 4 -t $CATKIN_WORKSPACE/src

# Dependencies not available in the Raspbian stable branch
mkdir -p $CATKIN_WORKSPACE/external_src
cd $CATKIN_WORKSPACE/external_src
sudo apt-get install -y checkinstall cmake

# libconsole-bridge-dev
sudo apt-get install -y libboost-system-dev libboost-thread-dev
git clone https://github.com/ros/console_bridge.git
cd console_bridge
cmake .
echo
echo "When check-install asks for any changes, the name (2) needs to change from"
echo "\"console-bridge\" to \"libconsole-bridge-dev\" otherwise the rosdep install"
echo "wont find it."
echo
sudo checkinstall make install
cd ..

# liblz4-dev
wget http://archive.raspbian.org/raspbian/pool/main/l/lz4/liblz4-1_0.0~r122-2_armhf.deb
wget http://archive.raspbian.org/raspbian/pool/main/l/lz4/liblz4-dev_0.0~r122-2_armhf.deb
sudo dpkg -i liblz4-1_0.0~r122-2_armhf.deb liblz4-dev_0.0~r122-2_armhf.deb

if [ "$ROS_VARIANT" == "desktop" ]; then
  # liburdfdom-headers-dev
  git clone https://github.com/ros/urdfdom_headers.git
  cd urdfdom_headers
  cmake .
  echo
  echo "When check-install asks for any changes, the name (2) needs to change from"
  echo "\"urdfdom-headers\" to \"liburdfdom-headers-dev\" otherwise the rosdep install"
  echo "wont find it."
  echo
  sudo checkinstall make install
  cd ..

  # liburdfdom-dev
  sudo apt-get install -y libboost-test-dev libtinyxml-dev
  git clone https://github.com/ros/urdfdom.git
  cd urdfdom
  cmake .
  echo
  echo "When check-install asks for any changes, the name (2) needs to change from"
  echo "\"urdfdom\" to \"liburdfdom-dev\" otherwise the rosdep install"
  echo "wont find it."
  echo
  sudo checkinstall make install
  cd ..

  # collada-dom-dev
  # Note: You will also need to patch collada_urdf as described here:
  #   https://github.com/ros/robot_model/issues/12
  #   https://groups.google.com/group/ros-sig-embedded/attach/1708811e0359ec39/0001-fixed-arm-build.patch?part=0.1
  # Also note: I had got the following error when building collada_parser:
  #   /usr/bin/ld: cannot find -lcollada-dom2.4-dp
  # To fix this, I ran the following command:
  #   sudo ln -s /usr/local/lib/libcollada-dom2.4-dp.so.2.4.0 /usr/local/lib/libcollada-dom2.4-dp.so
  sudo apt-get install -y libboost-filesystem-dev libxml2-dev
  wget http://downloads.sourceforge.net/project/collada-dom/Collada%20DOM/Collada%20DOM%202.4/collada-dom-2.4.0.tgz
  tar -xzf collada-dom-2.4.0.tgz
  cd collada-dom-2.4.0
  cmake .
  echo
  echo "When check-install asks for any changes, the name (2) needs to change from"
  echo "\"collada-dom\" to \"collada-dom-dev\" otherwise the rosdep install"
  echo "wont find it."
  echo
  sudo checkinstall make install
  cd ..

  # Building the collada_urdf package fails because Assimp is missing
  # http://answers.ros.org/question/48084/urdf_to_collada-undefined-reference-to-vtable-for-assimpiosystem/
  wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
  unzip assimp-3.1.1_no_test_models.zip
  cd assimp-3.1.1
  cmake .
  make
  sudo make install
  cd ..

  # Required fix for rviz:
  # http://answers.ros.org/question/52098/unable-to-compile-rviz-on-ubuntu-armhf/?answer=126851#post-id-126851
fi

# Resolving dependencies with rosdep
cd $CATKIN_WORKSPACE
rosdep install --from-paths $CATKIN_WORKSPACE/src --ignore-src --rosdistro $ROS_DISTRO -y -r --os=debian:$DEBIAN_VERSION

# rosdep will fail on Debian for several packages. They can be installed through pip:
#sudo pip install -U rosdep rospkg catkin-pkg

# Building the catkin Workspace
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space $INSTALL_LOCATION

# Add the following to .bashrc
source /opt/ros/indigo/setup.bash
