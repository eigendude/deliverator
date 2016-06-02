cmake_minimum_required(VERSION 2.8.3)
project(deliverator_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  smach_ros
  sensor_msgs
)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/controller
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#catkin_install_python(PROGRAMS nodes/controller_action
#                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
