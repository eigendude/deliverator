# Setting up networking

The robot's internal LAN can run in two modes: trusted and untrusted. Untrusted mode causes all nodes to firewall themselves from the network and communicate via OpenVPN. The untrusted threat model assumes an untrustworthy agent with access to the network but not the hardware (sensible when running a WiFi access point).

The robot defaults to a trusted network for performance reasons. Eventually, the robot will be able to switch between the two threat models at runtime.

## 1. Setting up the master node

The master node is responsible for hosting both networks and providing network services (DHCP, OpenVPN, possibly NetBIOS name resolution, possibly DSN).

Each network is implemented as a Linux network bridge. The network bridge acts like a router/switch: clients can connect and receive DHCP leases, and packets are switched between the clients. It's like having two virtual routers that exist only in software.

In general, most networking is handled by scripts and ROS nodes. The required manual labor is outlined below.

## 2. Creating the network bridges

First, add the following to `/etc/network/interfaces`:

```
auto br_untrusted
iface br_untrusted inet static
        address 10.9.8.1
        netmask 255.255.255.0
        bridge_ports none

auto br_trusted
iface br_trusted inet static
        address 10.11.12.1
        netmask 255.255.255.0
        bridge_ports none
```

This causes the master node to create two permanent, unconnected network bridges when booted. Network interfaces (like eth0) are added to the bridges at runtime.

The keyword `bridge_ports` defines the interfaces as bridges. For this extension to work, you'll need to install `bridge-utils`:

```shell
sudo apt-get install bridge-utils
```

The robot uses the names `br_trusted` and `br_untrusted` to identify the two bridges. The IP addresses are arbitrary and can be freely changed (adjust the instructions below accordingly).

## 3. Enabling DHCP on the bidges

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

The DHCP server keeps its current set of leases in `/var/lib/dhcp/dhcpd.leases`. This isn't scanned currently, but I'm writing this down for future reference.
