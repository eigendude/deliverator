cmake_minimum_required(VERSION 2.8.3)
project(deliverator_drivers)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

add_subdirectory(src/wifi)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
