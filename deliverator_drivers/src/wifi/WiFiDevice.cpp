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
#include "WiFiUtils.h"

#include "deliverator_msgs/WiFiChannelData.h"
#include "deliverator_msgs/WiFiStationData.h"
#include "ros/ros.h"

#include <linux/nl80211.h>
#include <net/if.h>
#include <netlink/genl/genl.h>
#include <netlink/attr.h>
#include <netlink/netlink.h>

using namespace deliverator;

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(ar)  (sizeof(ar) / sizeof(ar[0]))
#endif

#define BSS_IE_SSID  0

WiFiDevice::WiFiDevice(const std::string& name, NetlinkState& state) :
  m_name(name),
  m_state(state),
  m_scanner(state),
  m_callback(nullptr),
  m_sendCallback(nullptr),
  m_error(0)
{
}

void WiFiDevice::TriggerScan(bool passive, const std::vector<uint32_t>& channels, const std::vector<std::string>& ssids)
{
  NetlinkMsgPtr msg;
  if (!InitMsg(msg, NL80211_CMD_TRIGGER_SCAN))
    return;

  if (!passive)
  {
    if (!AddSsids(msg, ssids))
      return;
  }

  if (!channels.empty())
  {
    if (!AddChannels(msg, channels))
      return;
  }

  SendMsg(msg);
}

void WiFiDevice::WaitForScan()
{
  if (!m_scanner.ListenEvents())
    ROS_ERROR("Scan aborted!");
}

bool WiFiDevice::GetScanData(deliverator_msgs::WiFiInterfaceData& msg)
{
  bool bHasData = false;

  NetlinkMsgPtr netlinkMsg;
  if (!InitMsg(netlinkMsg, NL80211_CMD_GET_SCAN))
    return false;

  // Set up the callbacks
  nl_cb_set(m_callback, NL_CB_VALID, NL_CB_CUSTOM, StationHandler, this);

  SendMsg(netlinkMsg);

  P8PLATFORM::CLockObject lock(m_mutex);

  if (!m_stations.empty())
  {
    msg.name = m_name;

    for (auto it = m_stations.begin(); it != m_stations.end(); ++it)
    {
      deliverator_msgs::WiFiStationData stationMsg;
      it->second.GetStationData(stationMsg);
      msg.stations.emplace_back(std::move(stationMsg));
    }

    bHasData = true;
  }

  return bHasData;
}

void WiFiDevice::OnStation(const MacAddress& mac, const std::string& ssid, unsigned int channel, float dBm, uint8_t percent, unsigned int ageMs)
{
  P8PLATFORM::CLockObject lock(m_mutex);

  auto it = m_stations.find(mac);
  if (it != m_stations.end())
  {
    WiFiStation& station = it->second;
    station.UpdateIdentifiers(ssid, channel);
    station.UpdateMeasurements(dBm, percent, ageMs);
  }
  else
  {
    m_stations[mac] = WiFiStation(mac, ssid, channel, dBm, percent, ageMs);
  }
}

void WiFiDevice::OnFinish()
{
  m_error = 0;
}

void WiFiDevice::OnError(int nlmsgerr)
{
  m_error = nlmsgerr;
}

bool WiFiDevice::InitMsg(NetlinkMsgPtr& msg, nl80211_commands command)
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
  genlmsg_put(msg.get(), 0, 0, m_state.Get80211Id(), 0, NLM_F_DUMP, command, 0);

  // Add 32 bit integer attribute to the netlink message
  if (nla_put_u32(msg.get(), NL80211_ATTR_IFINDEX, interfaceIndex) != 0)
  {
    ROS_ERROR("Building message failed for %s", m_name.c_str());
    return false;
  }

  return true;
}

bool WiFiDevice::AddSsids(NetlinkMsgPtr& msg, const std::vector<std::string>& ssids) const
{
  // Build ssids msg
  NetlinkMsgPtr ssidsMsg = std::move(NetlinkMsgPtr(nlmsg_alloc(), FreeMessage));
  if (!ssids.empty())
  {
    int i = 1;
    for (auto& ssid : ssids)
    {
      if (nla_put(ssidsMsg.get(), i++, ssid.length(), ssid.c_str()) != 0)
      {
        ROS_ERROR("Building message failed for %s", m_name.c_str());
        return false;
      }
    }
  }
  else
  {
    if (nla_put(ssidsMsg.get(), 1, 0, "") != 0)
    {
      ROS_ERROR("Building message failed for %s", m_name.c_str());
      return false;
    }
  }

  nla_put_nested(msg.get(), NL80211_ATTR_SCAN_SSIDS, ssidsMsg.get());

  return true;
}

bool WiFiDevice::AddChannels(NetlinkMsgPtr& msg, const std::vector<uint32_t>& channels) const
{
  // Build freqs msg
  NetlinkMsgPtr freqsMsg = std::move(NetlinkMsgPtr(nlmsg_alloc(), FreeMessage));
  int i = 1;
  for (auto& channel : channels)
  {
    unsigned int freq = WiFiUtils::ChannelToFrequencyMHz(channel);
    if (freq == 0)
    {
      ROS_ERROR("Invalid channel for %s: %u", m_name.c_str(), channel);
      continue;
    }

    if (nla_put_u32(freqsMsg.get(), i++, freq) != 0)
    {
      ROS_ERROR("Building message failed for %s", m_name.c_str());
      return false;
    }
  }

  nla_put_nested(msg.get(), NL80211_ATTR_SCAN_FREQUENCIES, freqsMsg.get());

  return true;
}

void WiFiDevice::SendMsg(NetlinkMsgPtr& msg)
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
  nl_cb_set(m_callback, NL_CB_ACK, NL_CB_CUSTOM, AckHandler, this);

  // Receive a set of messages from the netlink socket
  while (this->m_error > 0)
    nl_recvmsgs(m_state.GetSocket(), m_callback);

  nl_cb_put(m_callback);

  m_callback = nullptr;
  m_sendCallback = nullptr;
}

int WiFiDevice::StationHandler(struct nl_msg* msg, void* arg)
{
  WiFiDevice* instance = static_cast<WiFiDevice*>(arg);
  if (!instance)
    return NL_STOP;

  WiFiStation station;

  struct nlattr* tb[NL80211_ATTR_MAX + 1];
  struct nlattr* bss[NL80211_BSS_MAX + 1];

  struct genlmsghdr* gnlh = static_cast<struct genlmsghdr*>(nlmsg_data(nlmsg_hdr(msg)));

  // Create attribute index based on stream of attributes.
  // Iterates over the stream of attributes and stores a pointer to each
  // attribute in the index array using the attribute type as index to the
  // array. Attribute with a type greater than the maximum type specified
  // will be silently ignored in order to maintain backwards compatibility.
  nla_parse(tb, NL80211_ATTR_MAX, genlmsg_attrdata(gnlh, 0), genlmsg_attrlen(gnlh, 0), nullptr);

  if (!tb[NL80211_ATTR_BSS])
  {
    ROS_DEBUG("Error on interface %s: BSS info missing!", instance->Name().c_str());
    return NL_SKIP;
  }

  static struct nla_policy bss_policy[NL80211_BSS_MAX + 1] = { };
  bss_policy[NL80211_BSS_TSF].type = NLA_U64;
  bss_policy[NL80211_BSS_FREQUENCY].type = NLA_U32;
  bss_policy[NL80211_BSS_BEACON_INTERVAL].type = NLA_U16;
  bss_policy[NL80211_BSS_CAPABILITY].type = NLA_U16;
  bss_policy[NL80211_BSS_SIGNAL_MBM].type = NLA_U32;
  bss_policy[NL80211_BSS_SIGNAL_UNSPEC].type = NLA_U8;
  bss_policy[NL80211_BSS_STATUS].type = NLA_U32;
  bss_policy[NL80211_BSS_SEEN_MS_AGO].type = NLA_U32;

  // Create attribute index based on nested attribute.
  // Feeds the stream of attributes nested into the specified attribute to nla_parse().
  if (nla_parse_nested(bss, NL80211_BSS_MAX, tb[NL80211_ATTR_BSS], bss_policy) != 0)
  {
    ROS_ERROR("Error on interface %s: Failed to parse nested attributes", instance->Name().c_str());
    return NL_SKIP;
  }

  // MAC address
  const uint8_t* macAddressBytes = static_cast<uint8_t*>(nla_data(bss[NL80211_BSS_BSSID]));
  if (macAddressBytes == nullptr)
    return NL_SKIP;

  MacAddress mac;
  for (unsigned int i = 0; i < ETH_ADDRESS_LEN; i++)
    mac[i] = macAddressBytes[i];

  // Frequency (MHz)
  unsigned int freqMHz = 0;
  if (bss[NL80211_BSS_FREQUENCY])
    freqMHz = nla_get_u32(bss[NL80211_BSS_FREQUENCY]);

  // This check is taken from iw
  if (freqMHz / 1000 > 45)
  {
    // TODO
    //is_dmg = true;
    return NL_SKIP;
  }

  // Signal strength (dBm)
  float dBm = 0.0f;
  if (bss[NL80211_BSS_SIGNAL_MBM])
  {
    int signalStrengthmBm = nla_get_u32(bss[NL80211_BSS_SIGNAL_MBM]);
    dBm = static_cast<float>(signalStrengthmBm) / 100;
  }

  // Signal strength in unspecified units (%)
  uint8_t percent = 0;
  if (bss[NL80211_BSS_SIGNAL_UNSPEC])
  {
    percent = nla_get_u8(bss[NL80211_BSS_SIGNAL_UNSPEC]);
  }

  // Age of this BSS entry (ms)
  unsigned int ageMs = 0;
  if (bss[NL80211_BSS_SEEN_MS_AGO])
  {
    ageMs = nla_get_u32(bss[NL80211_BSS_SEEN_MS_AGO]);
  }

  // SSID
  std::string ssid;
  if (bss[NL80211_BSS_INFORMATION_ELEMENTS])
  {
    unsigned char* ie = static_cast<unsigned char*>(nla_data(bss[NL80211_BSS_INFORMATION_ELEMENTS]));
    int ielen = nla_len(bss[NL80211_BSS_INFORMATION_ELEMENTS]);

    while (ielen >= 2 && ielen >= ie[1])
    {
      if (ie[0] == BSS_IE_SSID)
      {
        uint8_t len = ie[1];
        const uint8_t* data = ie + 2;
        ssid.assign(reinterpret_cast<const char*>(data), len);
        break;
      }
      ielen -= ie[1] + 2;
      ie += ie[1] + 2;
    }
  }

  unsigned int channel = WiFiUtils::FrequencyMHzToChannel(freqMHz);
  if (channel == 0)
    ROS_ERROR("Invalid frequency for %s: %u MHz", instance->Name().c_str(), freqMHz);

  instance->OnStation(mac, ssid, channel, dBm, percent, ageMs);

  return NL_SKIP;
}

int WiFiDevice::FinishHandler(struct nl_msg* msg, void* arg)
{
  static_cast<WiFiDevice*>(arg)->OnFinish();
  return NL_SKIP;
}

int WiFiDevice::AckHandler(struct nl_msg* msg, void* arg)
{
  static_cast<WiFiDevice*>(arg)->OnFinish();
  return NL_STOP;
}

int WiFiDevice::ErrorHandler(struct sockaddr_nl* nla, struct nlmsgerr* err, void* arg)
{
  static_cast<WiFiDevice*>(arg)->OnError(err->error);
  return NL_STOP;
}

void WiFiDevice::FreeMessage(struct nl_msg* msg)
{
  if (msg)
    nlmsg_free(msg);
}
