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

#include "WiFiStation.h"
#include "WiFiDeviceScanner.h"

#include "deliverator_msgs/WiFiInterfaceData.h"
#include "threads/mutex.h"

#include <atomic>
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

  typedef std::shared_ptr<struct nl_msg> NetlinkMsgPtr;

  class WiFiDevice
  {
  public:
    WiFiDevice(const std::string& name, NetlinkState& state);

    const std::string& Name() const { return m_name; }

    void TriggerScan(bool passive, const std::vector<uint32_t>& channels, const std::vector<std::string>& ssids);

    /*!
     * WARNING: A race condition may exist (see WiFiDeviceScanner.h). Call this
     * function immediately after TriggerScan();
     */
    void WaitForScan();

    bool GetScanData(deliverator_msgs::WiFiInterfaceData& msg);

  private:
    // Prevent copy
    inline WiFiDevice(const WiFiDevice& c);
    inline WiFiDevice& operator=(const WiFiDevice& c);

    bool InitMsg(NetlinkMsgPtr& msg, nl80211_commands command);
    bool AddSsids(NetlinkMsgPtr& msg, const std::vector<std::string>& ssids) const;
    bool AddChannels(NetlinkMsgPtr& msg, const std::vector<uint32_t>& channels) const;
    void SendMsg(NetlinkMsgPtr& msg);

    void OnStation(const MacAddress& mac, const std::string& ssid, unsigned int channel, float dBm, uint8_t percent, unsigned int ageMs);
    void OnFinish();
    void OnError(int nlmsgerr);

    static int StationHandler(struct nl_msg* msg, void* arg);
    static int FinishHandler(struct nl_msg* msg, void* arg);
    static int AckHandler(struct nl_msg* msg, void* arg);
    static int ErrorHandler(struct sockaddr_nl* nla, struct nlmsgerr* err, void* arg);

    static void FreeMessage(struct nl_msg* msg);

    std::string m_name;
    NetlinkState& m_state;
    WiFiDeviceScanner m_scanner;
    struct nl_cb* m_callback;
    struct nl_cb* m_sendCallback;
    std::atomic<int> m_error; // TODO: Switch to event
    P8PLATFORM::CEvent m_scanFinishedEvent;
    P8PLATFORM::CMutex m_mutex;
    std::map<MacAddress, WiFiStation> m_stations;
  };
}
