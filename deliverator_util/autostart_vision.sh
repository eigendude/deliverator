#! /bin/bash

# Add to /etc/rc.local:
#su <username> -c "<path_to_deliverator_util>/autostart_vision.sh"

source setup_env.sh

roslaunch deliverator_core vision.launch
