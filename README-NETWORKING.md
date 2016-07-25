# Setting up networking

The robot's internal LAN can run in two modes: trusted and untrusted. Untrusted mode causes all nodes to firewall themselves from the network and communicate via OpenVPN. The untrusted threat model assumes an untrustworthy agent with access to the network but not the hardware (sensible when running a WiFi access point).

The robot defaults to a trusted network for performance reasons. Eventually, the robot will be able to switch between the two threat models at runtime.

## 1. Setting up the master node

The master node is responsible for hosting both networks and providing network services (DHCP, OpenVPN, possibly NetBIOS name resolution, possibly DSN).

Each network is implemented as a Linux network bridge. The network bridge acts like a router/switch: clients can connect and receive DHCP leases, and packets are switched between the clients. It's like having two virtual routers that exist only in software.

In general, most networking is handled by scripts and ROS nodes. The required manual labor is outlined below.

### 1.1. Creating the network bridges

First, add the following to `/etc/network/interfaces`:

```
auto tap0
iface tap0 inet static
        pre-up ip tuntap add tap0 mode tap user root
        up ip link set dev tap0 up
        post-down ip link del dev tap0

auto br_untrusted
iface br_untrusted inet static
        address 10.9.8.1
        netmask 255.255.255.0
        bridge_ports none

auto br_trusted
iface br_trusted inet static
        address 10.11.12.1
        netmask 255.255.255.0
        bridge_ports tap0
```

The `tap0` interface is for OpenVPN connections. The next two interfaces are network bridges. Hardware interfaces (like `eth0`) are added to the untrusted bridge at runtime. When clients connect to OpenVPN, they are bridged to the trusted network through `tap0`.

The keyword `bridge_ports` defines the interfaces as bridges. For this extension to work, you'll need to install `bridge-utils`:

```shell
sudo apt-get install bridge-utils
```

The interface names `tap0`, `br_trusted` and `br_untrusted` are used by the robot. The IP addresses are arbitrary and can be freely changed (adjust the DHCP instructions below accordingly).

### 1.2. Enabling DHCP on the bidges

To enable DHCP on both bridges, add the following to `/etc/dhcp/dhcpd.conf`:

```
subnet 10.9.8.0 netmask 255.255.255.0 {
  range 10.9.8.2 10.9.8.254;
  option routers 10.9.8.1;
  option broadcast-address 10.9.8.255;
}

subnet 10.11.12.0 netmask 255.255.255.0 {
  range 10.11.12.2 10.11.12.254;
  option routers 10.11.12.1;
  option broadcast-address 10.11.12.255;
}
```

If you change the IP addresses in `/etc/network/interfaces`, the IP addresses should be changed here as well.

Next, add this to `/etc/default/isc-dhcp-server`:

```
INTERFACES="br_trusted br_untrusted"
```

The DHCP server keeps its current set of leases in `/var/lib/dhcp/dhcpd.leases`. This isn't scanned currently, but I'm writing this down for future reference.

### 1.3. Setting up OpenVPN

Setting up OpenVPN can be a pain, so I've provided a setup script: [setup_openvpn.sh](deliverator_util/setup_openvpn.sh). Run it from the `deliverator_util` directory. It follows the instructions below, so if something goes wrong the explanation can be found in this README.

If you're setting up OpenVPN manually, start by making sure OpenVPN is installed:

```shell
sudo apt-get install easy-rsa openvpn
```

Next, set up a certificate authority:

```shell
mkdir -p ~/openvpn_ws/easy-rsa
cp -r /usr/share/easy-rsa/* ~/openvpn_ws/easy-rsa
cd ~/openvpn_ws/easy-rsa
source vars
```

Define your certificate parameters on the command line to streamline the certificate generation process:

```shell
export KEY_COUNTRY="US"
export KEY_PROVINCE="CA"
export KEY_CITY="Los Angeles"
export KEY_ORG="Collection of Cowardly Cats"
export KEY_EMAIL="derikgalloway2FU@teleosaurs.xyz"
export KEY_CN=OpenVPN
export KEY_NAME=OpenVPN
export KEY_ALTNAMES=OpenVPN
export KEY_OU=OpenVPN
```

Initialize the key directory:

```shell
./clean-all
```

Build a root certificate:

```shell
./build-ca
```

Make a certificate/private key pair for the server using the root certificate (you can leave the challenge password blank):

```shell
./build-key-server server
```

Build Diffie-Hellman parameters for the server side of an SSL/TLS connection (takes about 10 minutes on the RPi 2):

```shell
./build-dh
```

Install keys:

```shell
cd keys/
sudo cp server.crt server.key ca.crt dh2048.pem /etc/openvpn/
cd ..
```

Create client certificates (you should substitute the client hostnames):

```shell
./build-key client1
./build-key client2
./build-key client3
./build-key client4
```

Download the [server configuration](deliverator_util/openvpn/server.conf):

```shell
cd /etc/openvpn
sudo wget https://raw.github.com/juztamau5/deliverator/master/deliverator_util/openvpn/server.conf
```

TODO: Add 'explicit-exit-notify' to client config files

Finally, start the server:

```shell
sudo service openvpn restart
```

If debugging is required, you can monitor the log using:

```shell
sudo tail --follow=name /etc/openvpn/openvpn.log
```
