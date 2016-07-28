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
#include "deliverator_msgs/CheckIsWireless.h"

#include <vector>

using namespace deliverator;

#define NODE_NAME "wifi_manager"
#define TOPIC_NAME "wifi_status"
#define SERVICE_NAME "check_is_wireless"

WiFiManager g_manager;

bool CheckIsWireless(deliverator_msgs::CheckIsWireless::Request& req,
                     deliverator_msgs::CheckIsWireless::Response& res)
{
  res.is_wireless = g_manager.IsWireless(req.device);
  return true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, NODE_NAME);

  if (!LinuxCapabilities::HasCapability(LinuxCapability::NET_ADMIN))
  {
    ROS_ERROR("Process does not have capability CAP_NET_ADMIN");
    return 1;
  }

  if (!g_manager.Initialize())
  {
    ROS_ERROR("Failed to initialize WiFi driver");
    return 1;
  }

  ros::NodeHandle n;
  ros::Publisher statusPub = n.advertise<deliverator_msgs::WiFiStatus>(TOPIC_NAME, 1);
  ros::ServiceServer service = n.advertiseService(SERVICE_NAME, CheckIsWireless);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std::vector<WiFiDevice> devices = g_manager.GetDevices();
    if (!devices.empty())
    {
      deliverator_msgs::WiFiStatus msg;

      for (auto& device : devices)
      {
        const std::string& ifaceName = device.Name();
        msg.data = "hello world"; // TODO
      }

      statusPub.publish(msg);
    }

    ros::spinOnce();

    loop_rate.sleep();
   }

   return 0;
}
