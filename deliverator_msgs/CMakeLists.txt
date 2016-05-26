cmake_minimum_required(VERSION 2.8.3)
project(deliverator_msgs)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
)

catkin_package()
