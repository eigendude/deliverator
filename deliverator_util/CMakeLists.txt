cmake_minimum_required(VERSION 2.8.3)
project(deliverator_util)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(PROGRAMS
    nodes/power_on.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
