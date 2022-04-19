cmake_minimum_required(VERSION 2.8.3)
project(spot_chicken_head)

find_package(catkin 
  REQUIRED
  COMPONENTS
  roscpp
  champ_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
  roscpp
  champ_msgs
  sensor_msgs
  geometry_msgs
  DEPENDS
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(pinocchio REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  include_directories(${EIGEN_INCLUDE_DIRS})
)

add_library(spot_chicken_head src/chicken_head.cpp src/math_helper.cpp)

add_executable(spot_chicken_head_node src/chicken_head_node.cpp)
target_link_libraries(spot_chicken_head_node chicken_head ${catkin_LIBRARIES} ${pinocchio_LIBRARIES})
add_dependencies(spot_chicken_head_node ${catkin_EXPORTED_TARGETS})
add_dependencies(spot_chicken_head ${catkin_EXPORTED_TARGETS})

install(TARGETS 
    spot_chicken_head
    spot_chicken_head_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)