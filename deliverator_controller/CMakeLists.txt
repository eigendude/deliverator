cmake_minimum_required(VERSION 2.8.3)
project(deliverator_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  smach_ros
)

catkin_package()

catkin_install_python(PROGRAMS nodes/controller.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#catkin_install_python(PROGRAMS nodes/controller_action.py
#                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
