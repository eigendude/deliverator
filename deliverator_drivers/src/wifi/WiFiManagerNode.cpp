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

#include "deliverator_msgs/CheckIsWireless.h"
#include "deliverator_msgs/StartScan.h"
#include "deliverator_msgs/EndScan.h"
#include "deliverator_msgs/WiFiScanData.h"
#include "ros/ros.h"

#include <memory>
#include <vector>

using namespace deliverator;

#define NODE_NAME                  "wifi_manager"
#define TOPIC_NAME                 "wifi_status"
#define CHECK_IS_WIRELESS_SERVICE  "check_is_wireless"
#define START_SCAN_SERVICE         "start_scan"
#define END_SCAN_SERVICE           "end_scan"

WiFiManager g_manager;

bool CheckIsWireless(deliverator_msgs::CheckIsWireless::Request& req,
                     deliverator_msgs::CheckIsWireless::Response& res)
{
  res.is_wireless = g_manager.IsWireless(req.device);
  return true;
}

bool StartScan(deliverator_msgs::StartScan::Request& req,
               deliverator_msgs::StartScan::Response& res)
{
  ROS_DEBUG("Starting %sscan on interface %s", req.passive ? "passive " : "", req.interface.c_str());
  g_manager.StartScan(req.interface, req.passive, req.channels, req.ssids);
  return true;
}

bool EndScan(deliverator_msgs::EndScan::Request& req,
               deliverator_msgs::EndScan::Response& res)
{
  ROS_DEBUG("Stopping scan on interface %s", req.interface.c_str());
  g_manager.EndScan(req.interface);
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

  ros::Publisher statusPub = n.advertise<deliverator_msgs::WiFiScanData>(TOPIC_NAME, 1);

  ros::ServiceServer services[] = {
      n.advertiseService(CHECK_IS_WIRELESS_SERVICE, CheckIsWireless),
      n.advertiseService(START_SCAN_SERVICE, StartScan),
      n.advertiseService(END_SCAN_SERVICE, EndScan),
  };

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    deliverator_msgs::WiFiScanData msg;
    if (g_manager.GetScanData(msg))
      statusPub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
   }

   return 0;
}
