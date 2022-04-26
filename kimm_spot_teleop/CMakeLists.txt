cmake_minimum_required(VERSION 2.8.3)
project(kimm_spot_teleop)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()
catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/kimm_spot_teleop.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

