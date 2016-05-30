cmake_minimum_required(VERSION 2.8.3)
project(deliverator_gazebo)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/ackermann_controller
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
