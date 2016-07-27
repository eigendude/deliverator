/*
 *      Copyright (C) 2016 juztamau5
 *
 *  This Program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This Program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this Program; see the file COPYING.  If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 */

#include "LinuxCapabilities.h"
#include "WiFiManager.h"

#include "ros/ros.h"
#include "deliverator_msgs/WiFiStatus.h"

#include <vector>

using namespace deliverator;

#define NODE_NAME "wifi_manager"
#define TOPIC_NAME "wifi_status"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, NODE_NAME);

  if (!LinuxCapabilities::HasCapability(LinuxCapability::NetworkAdmin))
  {
    ROS_ERROR("Process does not have capability CAP_SYS_ADMIN");
    return 1;
  }

  WiFiManager manager;
  if (!manager.Initialize())
  {
    ROS_ERROR("Failed to initialize WiFi driver");
    return 1;
  }

  ros::NodeHandle n;
  ros::Publisher statusPub = n.advertise<deliverator_msgs::WiFiStatus>(TOPIC_NAME, 1);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    deliverator_msgs::WiFiStatus msg;

    const std::vector<WiFiDevice>& devices = manager.GetDevices();
    for (auto& device : devices)
    {
      const std::string& ifaceName = "wlan0"; // TODO

      msg.data = "hello world"; // TODO
    }

    statusPub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
   }

   return 0;
}
