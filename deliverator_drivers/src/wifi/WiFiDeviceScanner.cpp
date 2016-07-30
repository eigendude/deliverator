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

#include "WiFiDeviceScanner.h"
#include "NetlinkState.h"

#include "ros/ros.h"

#include <linux/nl80211.h>
#include <netlink/msg.h>
#include <netlink/netlink.h>

using namespace deliverator;

WiFiDeviceScanner::WiFiDeviceScanner(NetlinkState& state) :
  m_state(state),
  m_bAborted(false),
  m_bFinished(false)
{
}

bool WiFiDeviceScanner::ListenEvents()
{
  m_bAborted = false;
  m_bFinished = false;

  struct nl_cb* cb = nl_cb_alloc(NL_CB_DEFAULT);
  if (cb == nullptr)
  {
    ROS_ERROR("Failed to allocate netlink callbacks");
    return false;
  }

  // No sequence checking for multicast messages
  nl_cb_set(cb, NL_CB_SEQ_CHECK, NL_CB_CUSTOM, NoSeqCheck, NULL);
  nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM, WaitEvent, NULL);

  while (!m_bAborted && !m_bFinished)
    nl_recvmsgs(m_state.GetSocket(), cb);

  nl_cb_put(cb);

  return !m_bAborted;
}

int WiFiDeviceScanner::WaitEvent(struct nl_msg* msg, void* arg)
{
  WiFiDeviceScanner* instance = static_cast<WiFiDeviceScanner*>(arg);

  struct genlmsghdr* gnlh = static_cast<struct genlmsghdr*>(nlmsg_data(nlmsg_hdr(msg)));

  switch (gnlh->cmd)
  {
  case NL80211_CMD_SCAN_ABORTED:
    instance->m_bAborted = true;
    break;
  case NL80211_CMD_NEW_SCAN_RESULTS:
    instance->m_bFinished = true;
    break;
  default:
    break;
  }

  return NL_SKIP;
}

int WiFiDeviceScanner::NoSeqCheck(struct nl_msg* msg, void *arg)
{
  return NL_OK;
}
