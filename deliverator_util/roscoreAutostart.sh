#! /bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Source ROS setup
for ros_distro in kinetic \
                  jade \
                  indigo \
; do
	SETUP_SCRIPT="/opt/ros/$ros_distro/setup.bash"
	if [ -f $SETUP_SCRIPT ]; then
		source $SETUP_SCRIPT
		break
	fi
done

# Source project script
source $SCRIPT_DIR/../../../devel/setup.bash

# Set params
export ROS_PACKAGE_PATH=$SCRIPT_DIR/..:$ROS_PACKAGE_PATH
export PATH=$PATH:$ROS_ROOT/bin

#roslaunch deliverator_core core.launch NAMESPACE:=deliverator
roscore
