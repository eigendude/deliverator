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

#include <memory>
#include <stdint.h>

#define ETH_ADDRESS_LEN  6

struct _cap_struct;
struct nl_cb;
struct nl_msg;

namespace deliverator
{
  class WiFiDevice;
  typedef std::shared_ptr<WiFiDevice> WiFiDevicePtr;

  typedef std::array<uint8_t, ETH_ADDRESS_LEN> MacAddress;

  typedef std::shared_ptr<struct _cap_struct> CapabilitiesPtr;
  typedef std::shared_ptr<struct nl_cb> NetlinkCallbackPtr;
  typedef std::shared_ptr<struct nl_msg> NetlinkMsgPtr;
}
