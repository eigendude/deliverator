#! /bin/bash

# Add to /etc/rc.local:
#su <username> -c "<path_to_deliverator_util>/autostart_client.sh"

source setup_env.sh

roslaunch deliverator_core autostart_client.launch
