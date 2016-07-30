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

#include "WiFiStation.h"

using namespace deliverator;

WiFiStation::WiFiStation() :
  m_channel(0),
  m_dBm(0.0f),
  m_percent(0),
  m_ageMs(0)
{
}

WiFiStation::WiFiStation(const MacAddress& mac,
                         const std::string& ssid,
                         unsigned int channel,
                         float dBm,
                         uint8_t percent,
                         unsigned int ageMs) :
  m_mac(mac),
  m_ssid(ssid),
  m_channel(channel),
  m_dBm(dBm),
  m_percent(percent),
  m_ageMs(ageMs)
{
}

void WiFiStation::UpdateIdentifiers(const std::string& ssid, unsigned int channel)
{
  m_ssid = ssid;
  m_channel = channel;
}

void WiFiStation::UpdateMeasurements(float dBm, uint8_t percent, unsigned int ageMs)
{
  m_dBm = dBm;
  m_percent = percent;
  m_ageMs = ageMs;
}

void WiFiStation::GetStationData(deliverator_msgs::WiFiStationData& msg) const
{
  msg.mac_address.assign(m_mac.begin(), m_mac.end());
  msg.ssid = m_ssid;
  msg.channel = m_channel;
  msg.dbm = m_dBm;
  msg.percent = m_percent;
  msg.age_ms = m_ageMs;
}
