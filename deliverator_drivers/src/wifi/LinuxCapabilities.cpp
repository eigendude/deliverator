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
#include <memory>
#include <sys/capability.h>
#include <sys/types.h>
#include <unistd.h>

using namespace deliverator;

namespace deliverator
{
  void FreeCapability(cap_t caps)
  {
    if (caps)
      cap_free(caps);
  }

  typedef std::unique_ptr<struct _cap_struct, void(*)(cap_t)> CapabilitiesPtr;
}

bool LinuxCapabilities::HasCapability(LinuxCapability cap)
{
  bool bHasCapability = false;

  cap_value_t capValue = TranslateCapability(cap);

  CapabilitiesPtr capabilities = CapabilitiesPtr(cap_get_pid(getpid()), FreeCapability);
  if (!capabilities)
  {
    ROS_ERROR("Failed to get capabilities of current process (cap = %d, errno = %d)", capValue, errno);
    return false;
  }

  if (cap_set_flag(capabilities.get(), CAP_EFFECTIVE, 1, &capValue, CAP_SET) != 0)
  {
    ROS_ERROR("Failed to set capabilities of current process (cap = %d, errno = %d)", capValue, errno);
    return false;
  }

  cap_flag_value_t result = CAP_CLEAR;
  if (cap_get_flag(capabilities.get(), capValue, CAP_EFFECTIVE, &result) != 0)
  {
    ROS_ERROR("Failed to read capabilities of current process (cap = %d, errno = %d)", capValue, errno);
    return false;
  }

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

  return bHasCapability;
}

int LinuxCapabilities::TranslateCapability(LinuxCapability cap)
{
  return static_cast<cap_value_t>(cap);
}
