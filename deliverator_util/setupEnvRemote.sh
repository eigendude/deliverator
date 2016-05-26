#!/bin/sh

COMPUTE_BOX_HOSTNAME="COMPUTE_BOX_HOSTNAME"

# Source this before you perform a distributed launch
export ROS_MASTER_URI=http://$COMPUTE_BOX_HOSTNAME:11311
export MASTER_HOSTNAME=$COMPUTE_BOX_HOSTNAME
export HOSTNAME=$(hostname)
export ROSLAUNCH_SSH_UNKNOWN=1
