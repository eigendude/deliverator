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

struct nl_cb;
struct nl_msg;
struct _cap_struct;

namespace deliverator
{
  void FreeCallback(struct nl_cb* msg);
  void FreeMessage(struct nl_msg* msg);
  void FreeCapability(struct _cap_struct* caps);

  class WiFiUtils
  {
  public:
    /*!
     * \brief Convert an IEEE 802.11 WLAN channel to its frequency in MHz
     */
    static unsigned int ChannelToFrequencyMHz(unsigned int channel);

    /*!
     * \brief Convert a frequency in MHz to its IEEE 802.11 WLAN channel number
     */
    static unsigned int FrequencyMHzToChannel(unsigned int freqMHz);
  };
}
