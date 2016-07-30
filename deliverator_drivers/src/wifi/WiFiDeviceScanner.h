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

struct nl_msg;

namespace deliverator
{
  class NetlinkState;

  class WiFiDeviceScanner
  {
  public:
    WiFiDeviceScanner(NetlinkState& state);

    /*
     *    Copyright (c) 2007, 2008    Johannes Berg
     *    Copyright (c) 2007          Andy Lutomirski
     *    Copyright (c) 2007          Mike Kershaw
     *    Copyright (c) 2008-2009     Luis R. Rodriguez
     *
     * Permission to use, copy, modify, and/or distribute this software for any
     * purpose with or without fee is hereby granted, provided that the above
     * copyright notice and this permission notice appear in all copies.
     *
     * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
     * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
     * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
     * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
     * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
     * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
     * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
     *
     */

    /*
     * WARNING: DO NOT USE THIS CODE IN YOUR APPLICATION (lol)
     *
     * This code has a bug, which requires creating a separate
     * nl80211 socket to fix:
     * It is possible for a NL80211_CMD_NEW_SCAN_RESULTS or
     * NL80211_CMD_SCAN_ABORTED message to be sent by the kernel
     * before (!) we listen to it, because we only start listening
     * after we send our scan request.
     *
     * Doing it the other way around has a race condition as well,
     * if you first open the events socket you may get a notification
     * for a previous scan.
     *
     * The only proper way to fix this would be to listen to events
     * before sending the command, and for the kernel to send the
     * scan request along with the event, so that you can match up
     * whether the scan you requested was finished or aborted (this
     * may result in processing a scan that another application
     * requested, but that doesn't seem to be a problem).
     *
     * Alas, the kernel doesn't do that (yet).
     */
    bool ListenEvents();

  private:
    static int WaitEvent(struct nl_msg* msg, void* arg);
    static int NoSeqCheck(struct nl_msg* msg, void* arg);

    NetlinkState& m_state;
    bool m_bAborted;
    bool m_bFinished;
  };
}
