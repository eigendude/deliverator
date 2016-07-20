#!/bin/sh

# Reference: http://www.howtogeek.com/115116/how-to-configure-ubuntus-built-in-firewall/

ufw --force enable || exit 1

# Allow SSH traffic on port 22
ufw allow 22/tcp

# Print the rules for the user to see
echo ""
ufw status

