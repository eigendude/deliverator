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

namespace deliverator
{
  void FreeMessage(struct nl_msg* msg)
  {
    if (msg)
      nlmsg_free(msg);
  }

  typedef std::unique_ptr<struct nl_msg, void(*)(struct nl_msg*)> MessagePtr;
}

bool WiFiManager::Initialize()
{
  // Allocate a new socket
  m_state = std::move(NetlinkState(nl_socket_alloc()));
  if (!m_state.IsValid())
  {
    ROS_ERROR("Failed to allocate netlink socket");
    return false;
  }

  nl_socket_set_buffer_size(m_state.GetSocket(), NETLINK_SOCKET_BUFFER_SIZE,
                                                 NETLINK_SOCKET_BUFFER_SIZE);

  if (genl_connect(m_state.GetSocket()) != 0)
  {
    ROS_ERROR("Failed to connect to generic netlink");
    Deinitialize();
    return false;
  }

  m_state.Set80211Id(genl_ctrl_resolve(m_state.GetSocket(), "nl80211"));
  if (!m_state.Is80211IdValid())
  {
    ROS_ERROR("nl80211 not found");
    Deinitialize();
    return false;
  }

  return true;
}

void WiFiManager::Deinitialize()
{
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
  P8PLATFORM::CLockObject lock(m_mutex);

  if (m_devices.find(interface) == m_devices.end())
    m_devices[interface] = std::make_shared<WiFiDevice>(interface, m_state);
}

void WiFiManager::EndScan(const std::string& interface)
{
  P8PLATFORM::CLockObject lock(m_mutex);

  auto it = m_devices.find(interface);
  if (it != m_devices.end())
    m_devices.erase(it);
}

bool WiFiManager::GetScanData(deliverator_msgs::WiFiScanData& msg)
{
  bool bHasData = false;

  P8PLATFORM::CLockObject lock(m_mutex);

  for (auto it = m_devices.begin(); it != m_devices.end(); ++it)
  {
    // TODO: Scan parameters
    it->second->TriggerScan(false, std::vector<uint32_t>(), std::vector<std::string>());

    //it->second->WaitForScan();

    deliverator_msgs::WiFiInterfaceData interface;
    if (it->second->GetScanData(interface))
    {
      msg.interfaces.emplace_back(std::move(interface));
      bHasData = true;
    }
  }

  return bHasData;
}
