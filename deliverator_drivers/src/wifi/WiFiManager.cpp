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

#include <net/if.h>
#include <netlink/genl/ctrl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/genl.h>
#include <netlink/netlink.h>

using namespace deliverator;

// Set to 1 to debug netlink
#define DEBUG_NETLINK  0

bool WiFiManager::Initialize()
{
  m_state.nl_sock = nl_socket_alloc();

  if (m_state.nl_sock == nullptr)
  {
    ROS_ERROR("Failed to allocate netlink socket");
    return false;
  }

  nl_socket_set_buffer_size(m_state.nl_sock, 8192, 8192);

  if (genl_connect(m_state.nl_sock) != 0)
  {
    ROS_ERROR("Failed to connect to generic netlink");
    nl_socket_free(m_state.nl_sock);
    return false;
  }

  m_state.nl80211_id = genl_ctrl_resolve(m_state.nl_sock, "nl80211");
  if (m_state.nl80211_id < 0)
  {
    ROS_ERROR("nl80211 not found");
    nl_socket_free(m_state.nl_sock);
    return false;
  }

  return true;
}

void WiFiManager::Deinitialize()
{
  nl_socket_free(m_state.nl_sock);
  m_state.nl_sock = nullptr;
}

std::vector<WiFiDevice> WiFiManager::GetDevices()
{
  P8PLATFORM::CLockObject lock(m_mutex);

  return m_devices;
}

bool WiFiManager::IsWireless(const std::string& interfaceName)
{
  unsigned int devidx = if_nametoindex(interfaceName.c_str());
  return devidx != 0;
}
