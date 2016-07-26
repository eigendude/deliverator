#!/bin/sh

# Add to /etc/rc.local:
#     <path_to_deliverator_util>/firewall_enable.sh

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

TRUSTED_IP=$($SCRIPT_DIR/get_ipv4_address "br-trusted")
UNTRUSTED_IP=$($SCRIPT_DIR/get_ipv4_address "br-untrusted")

ufw --force enable || exit 1

# Allow br-trusted
ufw allow in on br-trusted to $TRUSTED_IP

# Allow OpenVPN on br-untrusted
ufw allow in on br-untrusted to $UNTRUSTED_IP port 1194 proto udp

# Print the rules for the user to see
echo ""
ufw status
