#!/bin/bash

# Set to false to run roscore on the untrusted network
USE_TRUSTED_NETWORK=true

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
export PATH=$PATH:$ROS_ROOT/bin

if [ "$USE_TRUSTED_NETWORK" == "true" ]; then
    export ROS_IP=$($SCRIPT_DIR/get_ipv4_address "br-trusted")
else
    export ROS_IP=$($SCRIPT_DIR/get_ipv4_address "br-untrusted")
fi
