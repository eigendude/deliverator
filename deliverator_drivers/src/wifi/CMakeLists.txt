find_package(Libnl REQUIRED)
find_package(CAP REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LIBNL_INCLUDE_DIR}
  ${CAP_INCLUDE_DIRS}
)

add_executable(wifi_manager
  LinuxCapabilities.cpp
  NetlinkState.cpp
  WiFiDevice.cpp
  WiFiDeviceScanner.cpp
  WiFiManager.cpp
  WiFiManagerNode.cpp
  WiFiStation.cpp
  WiFiUtils.cpp
)

target_link_libraries(wifi_manager
  ${catkin_LIBRARIES}
  ${LIBNL_LIBRARIES}
  ${CAP_LIBRARIES}
)

add_dependencies(wifi_manager ${catkin_EXPORTED_TARGETS})
