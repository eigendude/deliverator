# Setting up the Raspberry Pi

The Raspberry Pis in this project use Raspbian based on Debian Jessie. If you're using Raspbian, these tips might come in handy.

## 1. Rename default user

Log in to the Raspberry Pi (password is "raspberry"):

```shell
ssh pi@192.168.1.2
```

Create temp user:

```shell
sudo useradd --create-home --gid sudo --shell /bin/bash temp
sudo passwd temp
```

Log out, then log in as temp:

```shell
exit
ssh temp@192.168.1.2
```

Rename default "pi" user:

```shell
sudo usermod --login <username> pi
sudo usermod --home /home/<username> --move-home <username>
```

Optional, rename the group:

```shell
sudo groupmod --new-name <groupname> pi
```

Log out, then log in with new username (password is still "raspberry"):

```shell
exit
ssh <username>@192.168.1.2
```

Don't forget to change your password and remove the temporary user:

```shell
passwd
sudo userdel --remove temp
```

Finally, erase the line `pi ALL=(ALL) NOPASSWD: ALL` from the sudoers file using visudo:

```shell
sudo visudo
```

## 2. Set timezone

First, check the current timezone:

```shell
date +'%:z %Z'
```

To change the timezone, use:

```shell
sudo dpkg-reconfigure tzdata
```

## 3. Install ROS from source

First, check if ROS can be installed from the package manager. This will save you a lot of time and space.

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
apt-cache search ros-kinetic-*
```

If `apt-cache search` returns results, you can follow the normal [Ubuntu installation steps](http://wiki.ros.org/kinetic/Installation/Ubuntu). Otherwise, you'll have to install from source.

Installing from source takes about 8.5 hours (when run by a script) and uses 3.7 GB above a vanilla Debian Jessie install (1 GB). 4.7 GB total means you'll need at least a 8 GB microSD card.

Before running the script, you should ensure that your **swap is big enough** and that your **sudo timeout is long enough**.

### 3.1. Upgrading packages

Before installing ROS, it is recommended that you upgrade your packages from the debian repository:

```shell
sudo apt-get update
sudo apt-get upgrade -y
sudo shutdown -r now
```

### 3.2. Increasing swap

Raspbian ships with a 100MB swap file. Compiling ROS with `-j4` on the Raspberry Pi 2 exceeds this limited space.

1GB of swap has been observed to be deficient. I had success with 8GB, the sweet spot is probably somewhere in the middle.

```shell
sudo swapoff --all
sudo dd if=/dev/zero of=/var/swap bs=1M count=8096
sudo mkswap /var/swap
sudo swapon /var/swap
```

This will enlarge the swap file to 8GB. Swapping to the microSD is slow, so you may want to put the swap file on external storage. I connected a SSD to a spare SATA-to-USB3 adapter and got 260 Mbps read/write over the RPi's USB controller.

### 3.3. Extending sudo timeout

The script takes several hours to complete, so to avoid interruptions it is recommended that you disable sudo timeouts until the script is finished:

```shell
sudo visudo
```

Add the following line to disabled timeouts:

```
Defaults  timestamp_timeout=-1
```

After the script is complete, you can set the timeout to something more reasonable, like 60 minutes:

```
Defaults  timestamp_timeout=60
```

### 3.4. Performing the install

A script has been added to the `deliverator_util` package to perform the entire install. It follows the instructions on the [Installing ROS Kinetic on the Raspberry Pi](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi) documentation article, so if the script breaks the offending line will be explained in the article.

Before downloading and blindly executing a script from the internet, view the source here: [raspbian-install-ros-kinetic.sh](https://github.com/juztamau5/deliverator/blob/master/deliverator_util/raspbian-install-ros-kinetic.sh). Then download and run:

```shell
wget https://raw.github.com/juztamau5/deliverator/master/deliverator_util/raspbian-install-ros-kinetic.sh
chmod +x raspbian-install-ros-kinetic.sh
time ./raspbian-install-ros-kinetic.sh
```

As mentioned before, this takes around 8.5 hours on the Raspberry Pi 2 and generates 3.7 GB worth of files. It really sucks when this script breaks and has to be restarted, so if you run into a problem please open an [issue](https://github.com/juztamau5/deliverator/issues) so I can update the script and the wiki.

### 3.5. Updating .bashrc

The environment is made aware of ROS by sourcing `/opt/ros/kinetic/setup.bash`. Add the following line to your `.bashrc`:

```shell
source /opt/ros/kinetic/setup.bash
```
