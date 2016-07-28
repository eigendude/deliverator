cmake_minimum_required(VERSION 2.8.3)
project(deliverator_msgs)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  std_msgs
  #sensor_msgs
  #geometry_msgs
  #diagnostic_msgs
  message_generation
)

add_message_files(FILES
  WheelSpeeds.msg
  WiFiStatus.msg
)

add_service_files(FILES
  CheckIsWireless.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)
