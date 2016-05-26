cmake_minimum_required(VERSION 2.8.3)
project(deliverator_core)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  image_transport
  pluginlib
  nodelet
  tf
)

find_package(cmake_modules REQUIRED)

catkin_python_setup()
