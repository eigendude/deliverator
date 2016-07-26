#! /bin/bash

# Add to /etc/rc.local:
#     su <username> -c "<path_to_deliverator_util>/autostart_server.sh"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source $SCRIPT_DIR/setup_env.sh

roslaunch deliverator_core autostart_server.launch
