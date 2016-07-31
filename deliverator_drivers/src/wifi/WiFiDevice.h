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

#include "deliverator_msgs/WiFiInterfaceData.h"
#include "deliverator_msgs/WiFiStationData.h"

#include <linux/nl80211.h>
#include <memory>
#include <stdint.h>
#include <string>
#include <vector>

struct nl_cb;
struct nl_msg;
struct nlmsgerr;
struct sockaddr_nl;

#define ETH_ADDRESS_LEN  6

namespace deliverator
{
  class NetlinkState;

  class WiFiDevice
  {
  public:
    WiFiDevice(const std::string& name, NetlinkState& state);

    const std::string& Name() const { return m_name; }

    bool TriggerScan(bool passive, const std::vector<uint32_t>& channels, const std::vector<std::string>& ssids);

    bool GetScanData(deliverator_msgs::WiFiInterfaceData& msg);

  private:
    // Prevent copy
    inline WiFiDevice(const WiFiDevice& c);
    inline WiFiDevice& operator=(const WiFiDevice& c);

    bool InitMsg(NetlinkMsgPtr& msg, nl80211_commands command);
    bool AddSsids(NetlinkMsgPtr& msg, const std::vector<std::string>& ssids) const;
    bool AddChannels(NetlinkMsgPtr& msg, const std::vector<uint32_t>& channels) const;
    bool SendMsg(NetlinkMsgPtr& msg);

    void OnStation(const MacAddress& mac, const std::string& ssid, unsigned int channel, float dBm, uint8_t percent, unsigned int ageMs);
    void OnFinish();
    void OnError(int error);

    static int StationHandler(struct nl_msg* msg, void* arg);
    static int FinishHandler(struct nl_msg* msg, void* arg);
    static int AckHandler(struct nl_msg* msg, void* arg);
    static int ErrorHandler(struct sockaddr_nl* nla, struct nlmsgerr* err, void* arg);

    std::string m_name;
    NetlinkState& m_state;

    NetlinkCallbackPtr m_callback;
    NetlinkCallbackPtr m_sendCallback;
    bool m_bError;
    bool m_bFinished;

    std::vector<deliverator_msgs::WiFiStationData> m_stations;
  };
}
