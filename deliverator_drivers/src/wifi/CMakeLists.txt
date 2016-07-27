include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(wifi_manager
  LinuxCapabilities.cpp
  WiFiDevice.cpp
  WiFiDeviceScanner.cpp
  WiFiManager.cpp
  WiFiManagerNode.cpp
)

target_link_libraries(wifi_manager ${catkin_LIBRARIES})

add_dependencies(wifi_manager deliverator_msgs)
