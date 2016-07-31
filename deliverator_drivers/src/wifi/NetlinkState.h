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

struct nl_sock;

namespace deliverator
{
  class NetlinkState
  {
  public:
    NetlinkState();
    NetlinkState(struct nl_sock* socket);
    ~NetlinkState() { Reset(); }

    NetlinkState& operator=(NetlinkState&& state);

    void Reset();

    struct nl_sock* GetSocket() { return m_socket; }
    int GetDriverId() const { return m_driverId; }

    void SetDriverId(int id) { m_driverId = id; }

    bool IsValid() const { return m_socket != nullptr; }
    bool IsDriverIdValid() const { return m_driverId >= 0; }

  private:
    struct nl_sock* m_socket;
    int m_driverId;
  };
}
