#!/bin/bash

# Choose which network to run roscore on:
#  - trusted - add firewall_enable.sh to rc.local
#  - untrusted - add firewall_disable.sh to rc.local
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

if [ "$1" == "server" ]; then
    if [ "$USE_TRUSTED_NETWORK" == "true" ]; then
        export ROS_IP=$($SCRIPT_DIR/get_ipv4_address br-trusted)
    else
        export ROS_IP=$($SCRIPT_DIR/get_ipv4_address br-untrusted)
    fi
elif [ "$1" == "client" ]; then
    export ROS_IP=$($SCRIPT_DIR/get_tap_address)
fi
