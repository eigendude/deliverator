#!/bin/sh

# Add to /etc/rc.local:
#     <path_to_deliverator_util>/firewall_enable.sh

ufw --force enable || exit 1

# Allow SSH
ufw allow 22/tcp

# Allow OpenVPN
ufw allow 1194/udp

# Print the rules for the user to see
echo ""
ufw status
