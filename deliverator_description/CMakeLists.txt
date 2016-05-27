cmake_minimum_required(VERSION 2.8.3)
project(deliverator_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)
