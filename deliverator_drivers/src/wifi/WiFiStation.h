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

#include "WiFiTypes.h"

#include "deliverator_msgs/WiFiStationData.h"
#include "ros/time.h"

#include <array>
#include <stdint.h>
#include <string>

namespace deliverator
{
  class WiFiStation
  {
  public:
    WiFiStation();
    WiFiStation(const MacAddress& mac,
                const std::string& ssid,
                unsigned int channel,
                float dBm,
                uint8_t percent,
                unsigned int ageMs);

    // Station identifiers
    const MacAddress& MAC() const { return m_mac; }
    const std::string& SSID() const { return m_ssid; }
    unsigned int Channel() const { return m_channel; }

    // Station measurements
    float SignalStrengthdBm() const { return m_dBm; }
    uint8_t SignalStrengthPercent() const { return m_percent; }
    const ros::Time& Timestamp() const { return m_timestamp; }

    // Update station measurements
    void UpdateIdentifiers(const std::string& ssid, unsigned int channel);
    void UpdateMeasurements(float dBm, uint8_t percent, unsigned int ageMs);

    // Serialize to ROS message
    void GetStationData(deliverator_msgs::WiFiStationData& msg) const;

  private:
    MacAddress m_mac;
    std::string m_ssid;
    unsigned int m_channel;

    float m_dBm;
    uint8_t m_percent;
    ros::Time m_timestamp;
  };
}
