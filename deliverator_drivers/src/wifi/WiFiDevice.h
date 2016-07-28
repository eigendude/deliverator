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

#include "deliverator_msgs/WiFiInterfaceData.h"
#include "threads/mutex.h"

#include <memory>
#include <string>
#include <vector>

namespace deliverator
{
  class WiFiDevice
  {
  public:
    WiFiDevice(const std::string& name);

    const std::string& Name() const { return m_name; }

    void StartScan(bool passive, const std::vector<uint32_t>& channels, const std::vector<std::string>& ssids);

    void EndScan();

    bool GetScanData(deliverator_msgs::WiFiInterfaceData& msg);

  private:
    // Prevent copy
    inline WiFiDevice(const WiFiDevice& c) { *this = c; }
    inline WiFiDevice& operator=(const WiFiDevice& c){ (void)c; return *this; }

    std::string m_name;
    P8PLATFORM::CEvent m_scanFinishedEvent;
  };
}
