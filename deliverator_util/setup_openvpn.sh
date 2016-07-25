#!/bin/bash

# Script to setup OpenVPN

OPENVPN_DIR=$HOME/openvpn_ws
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Install dependencies
sudo apt-get install -y easy-rsa openvpn

# Certificate authority setup
mkdir -p $OPENVPN_DIR/easy-rsa
cp -r /usr/share/easy-rsa/* $OPENVPN_DIR/easy-rsa/
cd $OPENVPN_DIR/easy-rsa
source vars

export KEY_COUNTRY="US"
export KEY_PROVINCE="CA"
export KEY_CITY="Los Angeles"
export KEY_ORG="Collection of Cowardly Cats"
export KEY_EMAIL="derikgalloway2FU@teleosaurs.xyz"
export KEY_CN=OpenVPN
export KEY_NAME=OpenVPN
export KEY_ALTNAMES=OpenVPN
export KEY_OU=OpenVPN

./clean-all
./build-ca

# Server certificates
./build-key-server server
time ./build-dh
cd keys/
sudo cp server.crt server.key ca.crt dh2048.pem /etc/openvpn/
cd ..

# Client certificates
./build-key client1
./build-key client2
./build-key client3
./build-key client4

# Server configuration
sudo cp $SCRIPT_DIR/openvpn/server.conf /etc/openvpn/

# Start the server
sudo service openvpn restart
