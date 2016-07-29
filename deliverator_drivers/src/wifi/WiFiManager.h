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

#include "NetlinkState.h"
#include "WiFiDevice.h"
#include "WiFiTypes.h"

#include "deliverator_msgs/WiFiScanData.h"
#include "threads/mutex.h"

#include <map>
#include <stdint.h>
#include <string>
#include <vector>

namespace deliverator
{
  class WiFiManager
  {
  public:
    WiFiManager() = default;
    ~WiFiManager() { Deinitialize(); }

    bool Initialize();
    void Deinitialize();

    bool IsWireless(const std::string& interfaceName);

    void StartScan(const std::string& interface, bool passive, const std::vector<uint32_t>& channels, const std::vector<std::string>& ssids);

    void EndScan(const std::string& interface);

    bool GetScanData(deliverator_msgs::WiFiScanData& msg);

  private:
    typedef std::map<std::string, WiFiDevicePtr> DeviceMap;

    NetlinkState m_state;
    DeviceMap m_devices;
    P8PLATFORM::CMutex m_mutex;
  };
}
