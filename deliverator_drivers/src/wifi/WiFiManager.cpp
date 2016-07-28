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

#include <netlink/genl/ctrl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/genl.h>
#include <netlink/netlink.h>

using namespace deliverator;

struct nl80211_state
{
  struct nl_sock* nl_sock;
  struct nl_cache* nl_cache;
  struct genl_family* nl80211;
};

bool WiFiManager::Initialize()
{
  nl80211_state state;

  state.nl_sock = nl_socket_alloc();

  if (state.nl_sock == nullptr)
  {
    fprintf(stderr, "Failed to allocate netlink socket.\n");
    return false;
  }

  nl_socket_set_buffer_size(state.nl_sock, 8192, 8192);

  if (genl_connect(state.nl_sock) != 0)
  {
    fprintf(stderr, "Failed to connect to generic netlink.\n");
    nl_socket_free(state.nl_sock);
    return false;
  }

  if (genl_ctrl_resolve(state.nl_sock, "nl80211") < 0)
  {
    fprintf(stderr, "nl80211 not found.\n");
    nl_socket_free(state.nl_sock);
    return false;
  }

  return true;
}

void WiFiManager::Deinitialize()
{
  // TODO
}

bool WiFiManager::IsWireless(const std::string& interfaceName) const
{
  for (auto& device : m_devices)
  {
    if (device.Name() == interfaceName)
      return true;
  }
  return false;
}

std::vector<WiFiDevice> WiFiManager::GetDevices() const
{
  return m_devices;
}
