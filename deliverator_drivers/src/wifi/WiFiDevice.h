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

struct nl_cb;
struct nl_msg;
struct nlmsgerr;
struct sockaddr_nl;

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

    bool IsScanning() const { return m_bIsScanning; }

    void WaitForScan() { } // TODO

    void EndScan();

    bool GetScanData(deliverator_msgs::WiFiInterfaceData& msg);

  private:
    // Prevent copy
    inline WiFiDevice(const WiFiDevice& c);
    inline WiFiDevice& operator=(const WiFiDevice& c);

    bool InitMsg(NetlinkMsgPtr& msg);
    void DeinitMsg(NetlinkMsgPtr& msg);

    int OnStation(struct nl_msg* msg);
    int OnFinish(struct nl_msg* msg);
    int OnError(struct sockaddr_nl *nla, struct nlmsgerr *err);

    static int StationHandler(struct nl_msg* msg, void* arg);
    static int FinishHandler(struct nl_msg* msg, void* arg);
    static int ErrorHandler(struct sockaddr_nl *nla, struct nlmsgerr *err, void* arg);

    static void FreeMessage(struct nl_msg* msg);

    std::string m_name;
    NetlinkState& m_state;
    bool m_bIsScanning;
    struct nl_cb* m_callback;
    struct nl_cb* m_sendCallback;
    int m_error; // TODO: Switch to event
    P8PLATFORM::CEvent m_scanFinishedEvent;
  };
}
