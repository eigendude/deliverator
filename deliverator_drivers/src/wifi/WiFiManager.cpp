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
    m_state.Reset();
    return false;
  }

  m_state.Set80211Id(genl_ctrl_resolve(m_state.GetSocket(), "nl80211"));
  if (!m_state.Is80211IdValid())
  {
    ROS_ERROR("nl80211 not found");
    m_state.Reset();
    return false;
  }

  return true;
}

void WiFiManager::Deinitialize()
{
  m_state.Reset();
}

std::vector<WiFiDevice> WiFiManager::GetDevices()
{
  P8PLATFORM::CLockObject lock(m_mutex);

  return m_devices;
}

bool WiFiManager::IsWireless(const std::string& interfaceName)
{
  bool bIsWireless = false;

  const unsigned int interfaceIndex = if_nametoindex(interfaceName.c_str());
  if (interfaceIndex == 0)
    return false; // Interface doesn't exist

  // Allocate a new netlink message
  MessagePtr msg = std::move(MessagePtr(nlmsg_alloc(), FreeMessage));
  if (!msg)
  {
    ROS_ERROR("Failed to allocate netlink message");
    return false;
  }

  // Allocate new callback handles
  nl_cb* cb = nl_cb_alloc(NL_CB_DEFAULT);
  nl_cb* s_cb = nl_cb_alloc(NL_CB_DEFAULT);
  if (cb == nullptr || s_cb == nullptr)
  {
    ROS_ERROR("Failed to allocate netlink callback handles");
    return false;
  }

  // Add generic netlink header to the netlink message
  genlmsg_put(msg.get(), 0, 0, m_state.Get80211Id(), 0, NLM_F_DUMP, NL80211_CMD_GET_STATION, 0);

  if (nla_put_u32(msg.get(), NL80211_ATTR_IFINDEX, interfaceIndex) != 0)
  {
    ROS_ERROR("Building message failed");
    return false;
  }

  return bIsWireless;
}
