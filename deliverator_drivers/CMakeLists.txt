cmake_minimum_required(VERSION 2.8.3)
project(deliverator_drivers)

catkin_package()

catkin_install_python(PROGRAMS
    nodes/wifi_manager
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
