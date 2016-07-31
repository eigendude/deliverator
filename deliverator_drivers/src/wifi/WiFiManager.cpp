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

#include "WiFiManager.h"

#include "ros/ros.h"

#include <linux/nl80211.h>
#include <net/if.h>
#include <netlink/genl/ctrl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/genl.h>
#include <netlink/attr.h>
#include <netlink/netlink.h>
#include <utility>

using namespace deliverator;

// Socket buffer size of a netlink socket for transmit and receive
// Providing a value of 0 assumes a good default value
#define NETLINK_SOCKET_BUFFER_SIZE  8192

// nl80811 driver name
#define DRIVER_NAME  "nl80211"

bool WiFiManager::Initialize()
{
  P8PLATFORM::CLockObject lock(m_stateMutex);

  // Allocate a new socket
  m_state = std::move(NetlinkState(nl_socket_alloc()));
  if (!m_state.IsValid())
  {
    ROS_ERROR("Failed to allocate netlink socket");
    return false;
  }

  nl_socket_set_buffer_size(m_state.GetSocket(), NETLINK_SOCKET_BUFFER_SIZE,
                                                 NETLINK_SOCKET_BUFFER_SIZE);

  // Create file descriptor and bind socket
  if (genl_connect(m_state.GetSocket()) != 0)
  {
    ROS_ERROR("Failed to connect to generic netlink");
    Deinitialize();
    return false;
  }

  m_state.SetDriverId(genl_ctrl_resolve(m_state.GetSocket(), DRIVER_NAME));
  if (!m_state.IsDriverIdValid())
  {
    ROS_ERROR("%s not found", DRIVER_NAME);
    Deinitialize();
    return false;
  }

  return true;
}

void WiFiManager::Deinitialize()
{
  P8PLATFORM::CLockObject lock(m_stateMutex);

  m_state.Reset();
}

bool WiFiManager::IsWireless(const std::string& interfaceName)
{
  bool bIsWireless = false;

  if (!interfaceName.empty())
    bIsWireless = (interfaceName[0] == 'w'); // TODO

  return bIsWireless;
}

void WiFiManager::StartScan(const std::string& interface, bool passive, const std::vector<uint32_t>& channels, const std::vector<std::string>& ssids)
{
  P8PLATFORM::CLockObject lock(m_deviceMutex);

  if (m_devices.find(interface) == m_devices.end())
    m_devices[interface] = std::make_shared<WiFiDevice>(interface, m_state);
  else
    ROS_WARN("Tried to start scan on %s, but it was already scanning!", interface.c_str());
}

void WiFiManager::EndScan(const std::string& interface)
{
  P8PLATFORM::CLockObject lock(m_deviceMutex);

  auto it = m_devices.find(interface);
  if (it != m_devices.end())
    m_devices.erase(it);
}

void WiFiManager::TriggerScans()
{
  DeviceMap devices;
  {
    P8PLATFORM::CLockObject lock(m_deviceMutex);
    devices = m_devices;
  }

  P8PLATFORM::CLockObject lock(m_stateMutex);

  for (auto it = devices.begin(); it != devices.end(); ++it)
  {
    // TODO: Scan parameters
    bool bPassive = false;
    std::vector<uint32_t> channels;
    std::vector<std::string> ssids;

    if (!it->second->TriggerScan(bPassive, channels, ssids))
    {
      const std::string& interfaceName = it->first;
      ROS_WARN("Failed to trigger scan on %s, closing interface", interfaceName.c_str());
      EndScan(interfaceName);
    }
  }
}

bool WiFiManager::GetScanData(deliverator_msgs::WiFiScanData& msg)
{
  bool bHasData = false;

  DeviceMap devices;
  {
    P8PLATFORM::CLockObject lock(m_deviceMutex);
    devices = m_devices;
  }

  P8PLATFORM::CLockObject lock(m_stateMutex);

  for (auto it = devices.begin(); it != devices.end(); ++it)
  {
    deliverator_msgs::WiFiInterfaceData interface;
    if (it->second->GetScanData(interface))
    {
      msg.interfaces.emplace_back(std::move(interface));
      bHasData = true;
    }
  }

  return bHasData;
}
