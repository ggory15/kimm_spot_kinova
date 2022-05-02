cmake_minimum_required(VERSION 2.8.3)
project(spot_kinova_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  sensor_msgs
  std_msgs
  actionlib_msgs
  actionlib
)

find_package(Eigen3 REQUIRED)

# add_message_files(
#   FILES
#   simple.msg
# )

add_action_files(DIRECTORY action FILES 
  JointPosture.action
  SE3.action
  Walk.action
  WalkSimulation.action
  BodyPosture.action
  Wholebody.action
  PredefinedPosture.action
)

# add_service_files(
#   FILES
#   IdleControl.srv
#   ObjectConnection.srv
#   SetTrajectoryFollowerGain.srv
#   GetObjectPoseArray.srv
#   GetObjectPose.srv
#   GetObjectPose_with_points.srv
#   SetSpiralGain.srv
#   GetOccupancyVoxel.srv
# )

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs actionlib_msgs sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
  geometry_msgs
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)