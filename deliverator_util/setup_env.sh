#!/bin/bash

# Set to false to run roscore on the untrusted network
USE_TRUSTED_NETWORK=true

# TODO: Find a better place for this
if [ "$USE_TRUSTED_NETWORK" == "true" ]; then
    # TODO: Add firewall on br_untrusted
    # TODO: Disable firewall on br_trusted
else
    # TODO: Disable firewall on br_untrusted
fi

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
    export ROS_IP=$($SCRIPT_DIR/get_ipv4_address "br_trusted")
else
    export ROS_IP=$($SCRIPT_DIR/get_ipv4_address "br_untrusted")
fi
