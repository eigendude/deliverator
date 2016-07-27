cmake_minimum_required(VERSION 2.8.3)
project(deliverator_drivers)

find_package(catkin REQUIRED COMPONENTS roscpp)

include(CheckCXXCompilerFlag)

# Check for C++11
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support")
endif()

catkin_package()

add_subdirectory(src/wifi)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
