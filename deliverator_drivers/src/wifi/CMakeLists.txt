find_package(Libnl REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LIBNL_INCLUDE_DIR}
)

add_executable(wifi_manager
  LinuxCapabilities.cpp
  WiFiDevice.cpp
  WiFiDeviceScanner.cpp
  WiFiManager.cpp
  WiFiManagerNode.cpp
)

target_link_libraries(wifi_manager
  ${catkin_LIBRARIES}
  ${LIBNL_LIBRARIES}
)

add_dependencies(wifi_manager ${catkin_EXPORTED_TARGETS})
