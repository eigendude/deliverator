cmake_minimum_required(VERSION 2.8.3)
project(deliverator_networking)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
  nodes/network_client
  nodes/network_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
