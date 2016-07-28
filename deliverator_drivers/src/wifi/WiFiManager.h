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
#pragma once

#include "WiFiDevice.h"

#include "deliverator_msgs/WiFiStatus.h"
#include "threads/mutex.h"

#include <vector>

struct nl80211_state
{
  struct nl_sock* nl_sock;
  int nl80211_id;
};

namespace deliverator
{
  class WiFiManager
  {
  public:
    ~WiFiManager() { Deinitialize(); }

    bool Initialize();
    void Deinitialize();

    std::vector<WiFiDevice> GetDevices();

    bool IsWireless(const std::string& interfaceName);

  private:
    nl80211_state m_state;
    std::vector<WiFiDevice> m_devices;
    P8PLATFORM::CMutex m_mutex;
  };
}
