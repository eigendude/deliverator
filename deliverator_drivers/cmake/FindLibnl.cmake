# - Find libnl
#
# This module defines
#  LIBNL_FOUND - whether the libnl library was found
#  LIBNL_LIBRARIES - the libnl library
#  LIBNL_INCLUDE_DIR - the include path of the libnl library

if(NOT LIBNL_FIND_QUIETLY)
  message(STATUS "Searching for Linux netlink library")
endif()

find_path(LIBNL_INCLUDE_DIR
  NAMES
  netlink/netlink.h
  PATH_SUFFIXES
  libnl3
)

find_library(LIBNL_LIBRARY NAMES nl nl-3)
find_library(LIBNL_GENL_LIBRARY NAMES nl-genl nl-genl-3)
find_library(LIBNL_NETFILTER_LIBRARY NAMES nl-nf nl-nf-3)
find_library(LIBNL_ROUTE_LIBRARY NAMES rl-route nl-route-3)

if (LIBNL_INCLUDE_DIR AND LIBNL_LIBRARY)
  set(LIBNL_FOUND TRUE)
endif()

if(LIBNL_FOUND)
  set(LIBNL_LIBRARIES
      ${LIBNL_LIBRARY}
      ${LIBNL_GENL_LIBRARY}
      ${LIBNL_NETFILTER_LIBRARY}
      ${LIBNL_ROUTE_LIBRARY}
  )
  if(NOT LIBNL_FIND_QUIETLY)
    message("Found netlink includes: ${LIBNL_INCLUDE_DIR}")
    message("Found netlink libraries:  ${LIBNL_LIBRARIES}")
  endif()
else()
  if(NOT LIBNL_FIND_QUIETLY)
    message("Netlink version 3 development packages cannot be found.")
    message("In Debian/Ubuntu, they may be called:")
    message("libnl-3-dev libnl-genl-3-dev libnl-nf-3-dev libnl-route-3-dev")
  endif()
endif()
