cmake_minimum_required(VERSION 3.0.2)
project(vrep_creative_stt)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/vrep_creative_stt_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)