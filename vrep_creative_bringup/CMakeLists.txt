cmake_minimum_required(VERSION 3.0.2)
project(vrep_creative_bringup)

find_package(catkin REQUIRED COMPONENTS
  rospy
  urdf
  tf
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  rospy
  tf
  urdf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
  src/vrep_creative_bringup/vrep_creative_bringup_simulator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)