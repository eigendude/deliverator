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

#include "WiFiDevice.h"

using namespace deliverator;

WiFiDevice::WiFiDevice(const std::string& name) :
  m_name(name)
{
}

void WiFiDevice::StartScan(bool passive, const std::vector<uint32_t>& channels, const std::vector<std::string>& ssids)
{
  // TODO
}

void WiFiDevice::EndScan()
{
  // TODO
}

bool WiFiDevice::GetScanData(deliverator_msgs::WiFiInterfaceData& msg)
{
  bool bHasData = false;

  msg.name = m_name;
  bHasData = true; // TODO

  return bHasData;
}