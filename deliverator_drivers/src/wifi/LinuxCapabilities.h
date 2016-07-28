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

namespace deliverator
{
  // From sys/capabilities.h
  // Do not change enum values
  enum class LinuxCapability
  {
    CHOWN            = 0,
    DAC_OVERRIDE     = 1,
    DAC_READ_SEARCH  = 2,
    FOWNER           = 3,
    FSETID           = 4,
    KILL             = 5,
    SETGID           = 6,
    SETUID           = 7,
    SETPCAP          = 8,
    LINUX_IMMUTABLE  = 9,
    NET_BIND_SERVICE = 10,
    NET_BROADCAST    = 11,
    NET_ADMIN        = 12,
    NET_RAW          = 13,
    IPC_LOCK         = 14,
    IPC_OWNER        = 15,
    SYS_MODULE       = 16,
    SYS_RAWIO        = 17,
    SYS_CHROOT       = 18,
    SYS_PTRACE       = 19,
    SYS_PACCT        = 20,
    SYS_ADMIN        = 21,
    SYS_BOOT         = 22,
    SYS_NICE         = 23,
    SYS_RESOURCE     = 24,
    SYS_TIME         = 25,
    SYS_TTY_CONFIG   = 26,
    MKNOD            = 27,
    LEASE            = 28,
    AUDIT_WRITE      = 29,
    AUDIT_CONTROL    = 30,
    SETFCAP          = 31,
    MAC_OVERRIDE     = 32,
    MAC_ADMIN        = 33,
    SYSLOG           = 34,
    WAKE_ALARM       = 35,
    BLOCK_SUSPEND    = 36,
    AUDIT_READ       = 37,
  };

  class LinuxCapabilities
  {
  public:
    static bool HasCapability(LinuxCapability cap);

  private:
    static int TranslateCapability(LinuxCapability cap);
  };
}
