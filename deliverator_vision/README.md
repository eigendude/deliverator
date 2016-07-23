# Deliverator Vision

This package contains camera and depth sensor drivers.

## Setup Instructions

This package can be run on nodes with little processing power, such as the Raspberry Pi Zero. The setup instructions assume that the catkin workspace is set up like the top-level [README.md](../README.md).

If the node is exclusively for video drivers, the other packages can be suppressed to avoid pulling in unnecessary dependencies:

```shell
cd ~/deliverator_ws/src/deliverator
git config core.sparsecheckout true
echo deliverator_vision/ > .git/info/sparse-checkout
git checkout
```

## Dependencies

This package depends on the gscam driver, which is not packaged for debian yet. Clone it manually:

```shell
cd ~/deliverator_ws/src
git clone https://github.com/ros-drivers/gscam.git
```

If you're running on a Raspberry Pi, then the `image_common` package that gscam depends on isn't avaiable. Clone it so that we can build it from source:

```shell
git clone https://github.com/ros-perception/image_common.git
```

Once these dependencies are in place, their system depends can be installed with rosdep:

```shell
rosdep install -y --from-paths . --ignore-src --rosdistro kinetic -r --os=debian:jessie
```

## Compilation

To compile run `catkin_make` from the catkin workspace folder.

```shell
cd ~/deliverator_ws
catkin_make
```
