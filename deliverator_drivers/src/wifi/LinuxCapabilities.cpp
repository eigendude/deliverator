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

#include "LinuxCapabilities.h"

#include "ros/ros.h"

#include <errno.h>
#include <sys/capability.h>
#include <sys/types.h>
#include <unistd.h>

using namespace deliverator;

bool LinuxCapabilities::HasCapability(LinuxCapability cap)
{
  bool bHasCapability = false;

  cap_t capabilities = cap_get_pid(getpid());
  if (capabilities == nullptr)
  {
    ROS_ERROR("Failed to get capabilities of current process (errno = %d)", errno);
  }
  else
  {
    cap_flag_value_t result;
    if (cap_get_flag(capabilities, TranslateCapability(cap), CAP_EFFECTIVE, &result) == 0)
    {
      switch (result)
      {
      case CAP_CLEAR:
        bHasCapability = false;
        break;
      case CAP_SET:
        bHasCapability = true;
        break;
      default:
        break;
      }
    }
    cap_free(capabilities);
  }

  return false;
}

int LinuxCapabilities::TranslateCapability(LinuxCapability cap)
{
  switch (cap)
  {
    case LinuxCapability::NetworkAdmin: return CAP_SYS_ADMIN;
    default:
      break;
  }

  return 0;
}
