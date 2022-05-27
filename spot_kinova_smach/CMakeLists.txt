cmake_minimum_required(VERSION 2.8.3)
project(spot_kinova_smach)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  smach_ros
  std_srvs
)

catkin_python_setup()
catkin_package()
