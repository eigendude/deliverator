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

#include "NetlinkState.h"

#include <netlink/netlink.h>

using namespace deliverator;

NetlinkState::NetlinkState() :
  m_socket(nullptr),
  m_driverId(-1)
{
}

NetlinkState::NetlinkState(struct nl_sock* socket) :
  m_socket(socket),
  m_driverId(-1)
{
}

NetlinkState& NetlinkState::operator=(NetlinkState&& state)
{
  m_socket = state.m_socket;
  state.m_socket = nullptr;

  m_driverId = state.m_driverId;

  return *this;
}

void NetlinkState::Reset()
{
  if (m_socket != nullptr)
  {
    nl_socket_free(m_socket);
    m_socket = nullptr;
  }
}
