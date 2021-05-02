cmake_minimum_required(VERSION 3.0.2)
project(vrep_creative_cmd)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  roscpp
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/vrep_creative_cmd_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)