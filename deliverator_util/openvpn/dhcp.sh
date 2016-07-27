#!/bin/bash

# From thread "Openvpn Bridged with DHCP via remote LAN":
#     http://ubuntuforums.org/showthread.php?t=2156126
#
# Fix for bug in OpenVPN where IP address is not given to tap interface.

[ -x /sbin/dhclient ] || exit 0

case $script_type in
  up)
    dhclient -v "${dev}" &
    ;;
  down)
    echo "Releasing ${dev} DHCP lease."
    dhclient -r "${dev}"
    ;;
esac
