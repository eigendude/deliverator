cmake_minimum_required(VERSION 2.8.3)
project(deliverator_core)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rospack
  diagnostic_msgs
  deliverator_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
    nodes/device_manager
    nodes/diagnostics
    nodes/network_client
    nodes/network_server
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
