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
#include "NetlinkState.h"

#include "ros/ros.h"

#include <linux/nl80211.h>
#include <net/if.h>
#include <netlink/genl/genl.h>
#include <netlink/attr.h>
#include <netlink/netlink.h>

using namespace deliverator;

WiFiDevice::WiFiDevice(const std::string& name, NetlinkState& state) :
  m_name(name),
  m_state(state),
  m_bIsScanning(false),
  m_callback(nullptr),
  m_sendCallback(nullptr),
  m_error(0)
{
}

void WiFiDevice::TriggerScan(bool passive, const std::vector<uint32_t>& channels, const std::vector<std::string>& ssids)
{
  NetlinkMsgPtr msg;
  if (!InitMsg(msg))
    return;

  if (!passive)
  {
    // TODO: If not passive, build ssids msg
  }
  // TODO: Build freqs msg

  SendMsg(msg, false);

  m_bIsScanning = true;
}

void WiFiDevice::EndScan()
{
  m_bIsScanning = false;
}

bool WiFiDevice::GetScanData(deliverator_msgs::WiFiInterfaceData& msg)
{
  bool bHasData = false;

  NetlinkMsgPtr netlinkMsg;
  if (!InitMsg(netlinkMsg))
    return false;

  // Set up the callbacks
  nl_cb_set(m_callback, NL_CB_VALID, NL_CB_CUSTOM, StationHandler, this);

  SendMsg(netlinkMsg, true);

  msg.name = m_name;
  bHasData = true; // TODO

  return bHasData;
}

int WiFiDevice::OnStation(struct nl_msg* msg)
{
  // TODO
  return NL_SKIP;
}

int WiFiDevice::OnFinish(struct nl_msg* msg)
{
  m_error = 0; // TODO
  return NL_SKIP;
}

int WiFiDevice::OnError(struct sockaddr_nl *nla, struct nlmsgerr *err)
{
  m_error = err->error; // TODO
  return NL_STOP;
}

int WiFiDevice::StationHandler(struct nl_msg* msg, void* arg)
{
  return static_cast<WiFiDevice*>(arg)->OnStation(msg);
}

int WiFiDevice::FinishHandler(struct nl_msg* msg, void* arg)
{
  return static_cast<WiFiDevice*>(arg)->OnFinish(msg);
}

int WiFiDevice::ErrorHandler(struct sockaddr_nl *nla, struct nlmsgerr *err, void* arg)
{
  return static_cast<WiFiDevice*>(arg)->OnError(nla, err);
}

bool WiFiDevice::InitMsg(NetlinkMsgPtr& msg)
{
  const unsigned int interfaceIndex = if_nametoindex(m_name.c_str());
  if (interfaceIndex == 0)
    return false; // Interface doesn't exist

  // Allocate a new netlink message
  msg = std::move(NetlinkMsgPtr(nlmsg_alloc(), FreeMessage));
  if (!msg)
  {
    ROS_ERROR("Failed to allocate netlink message for %s", m_name.c_str());
    return false;
  }

  // Allocate new callback handles
  m_callback = nl_cb_alloc(NL_CB_DEFAULT);
  m_sendCallback = nl_cb_alloc(NL_CB_DEFAULT);
  if (m_callback == nullptr || m_sendCallback == nullptr)
  {
    ROS_ERROR("Failed to allocate netlink callback handles for %s", m_name.c_str());
    return false;
  }

  // Add generic netlink header to the netlink message
  genlmsg_put(msg.get(), 0, 0, m_state.Get80211Id(), 0, NLM_F_DUMP, NL80211_CMD_GET_STATION, 0);

  // Add 32 bit integer attribute to the netlink message
  if (nla_put_u32(msg.get(), NL80211_ATTR_IFINDEX, interfaceIndex) != 0)
  {
    ROS_ERROR("Building message failed for %s", m_name.c_str());
    return false;
  }

  return true;
}

void WiFiDevice::SendMsg(NetlinkMsgPtr& msg, bool bWait)
{
  // Set up the callbacks
  nl_socket_set_cb(m_state.GetSocket(), m_sendCallback);

  // Automatically complete and send the netlink message
  if (nl_send_auto_complete(m_state.GetSocket(), msg.get()) < 0)
  {
    ROS_ERROR("Failed to send netlink message for %s", m_name.c_str());
    return;
  }

  m_error = 1;

  nl_cb_err(m_callback, NL_CB_CUSTOM, ErrorHandler, this);
  nl_cb_set(m_callback, NL_CB_FINISH, NL_CB_CUSTOM, FinishHandler, this);

  // Receive a set of messages from the netlink socket
  if (bWait)
  {
    while (this->m_error > 0)
      nl_recvmsgs(m_state.GetSocket(), m_callback);
  }

  nl_cb_put(m_callback);

  m_callback = nullptr;
  m_sendCallback = nullptr;
}

void WiFiDevice::FreeMessage(struct nl_msg* msg)
{
  if (msg)
    nlmsg_free(msg);
}
