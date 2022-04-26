cmake_minimum_required(VERSION 3.0.2)
project(kimm_spot_kinova)


add_compile_options(-std=c++17 -Ofast)

INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/ide.cmake)
INCLUDE(cmake/apple.cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}" )

OPTION(EIGEN_RUNTIME_NO_MALLOC "If ON, it can assert in case of runtime allocation" ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  rospy
  roslib
  spot_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(kimm_trajectory_smoother REQUIRED)
find_package(eiquadprog REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    geometry_msgs
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
  DEPENDS system_lib
)
link_directories(${Boost_LIBRARY_DIR})

###########
## Build ##
###########

## Define OS Type
add_definitions(-D_OS_UNIX)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${kimm_trajectory_smoother_INCLUDE_DIRS}
  ${eiquadprog_INCLUDE_DIRS}
)

SET(${PROJECT_NAME}_HEADERS
  include/kimm_spot_kinova/config.hpp

  include/kimm_spot_kinova/robots/fwd.hpp
  include/kimm_spot_kinova/robots/robot-wrapper.hpp

  include/kimm_spot_kinova/math/fwd.hpp
  include/kimm_spot_kinova/math/constraint_base.hpp
  include/kimm_spot_kinova/math/constraint_equality.hpp
  include/kimm_spot_kinova/math/constraint_inequality.hpp
  include/kimm_spot_kinova/math/constraint_bound.hpp
  include/kimm_spot_kinova/math/util.hpp

  include/kimm_spot_kinova/trajectories/fwd.hpp
  include/kimm_spot_kinova/trajectories/trajectory_base.hpp
  include/kimm_spot_kinova/trajectories/trajectory_euclidian.hpp
  include/kimm_spot_kinova/trajectories/trajectory_se3.hpp

  include/kimm_spot_kinova/tasks/fwd.hpp
  include/kimm_spot_kinova/tasks/task_base.hpp
  include/kimm_spot_kinova/tasks/task_joint_posture.hpp
  include/kimm_spot_kinova/tasks/task_motion.hpp
  include/kimm_spot_kinova/tasks/task_se3_equality.hpp

  include/kimm_spot_kinova/util/statistics.hpp
  include/kimm_spot_kinova/util/Stdafx.hh
  include/kimm_spot_kinova/util/stop_watch.hpp

  include/kimm_spot_kinova/solver/fwd.hpp
  include/kimm_spot_kinova/solver/util.hpp
  include/kimm_spot_kinova/solver/solver_HQP_output.hpp
  include/kimm_spot_kinova/solver/solver_HQP_base.hpp
  include/kimm_spot_kinova/solver/solver_HQP_factory.hpp
  include/kimm_spot_kinova/solver/solver_HQP_factory.hxx
  include/kimm_spot_kinova/solver/solver_HQP_eiquadprog.hpp
  include/kimm_spot_kinova/solver/solver_HQP_eiquadprog_rt.hpp
  include/kimm_spot_kinova/solver/solver_HQP_eiquadprog_rt.hxx
  include/kimm_spot_kinova/solver/solver_HQP_eiquadprog_fast.hpp

  include/kimm_spot_kinova/formulation/inverse_dynamics_formulation_base.hpp
  include/kimm_spot_kinova/formulation/inverse_dynamics_formulation_vel.hpp

)

SET(${PROJECT_NAME}_SOURCES
  src/robots/robot-wrapper.cpp

  src/math/constraint_base.cpp
  src/math/constraint_equality.cpp
  src/math/constraint_inequality.cpp
  src/math/constraint_bound.cpp
  src/math/util.cpp

  src/trajectories/trajectory_euclidian.cpp
  src/trajectories/trajectory_se3.cpp

  src/tasks/task_base.cpp
  src/tasks/task_motion.cpp
  src/tasks/task_joint_posture.cpp
  src/tasks/task_se3_equality.cpp
  
  src/util/statistics.cpp
  src/util/stop_watch.cpp
  
  src/solver/solver_HQP_base.cpp
  src/solver/solver_HQP_factory.cpp
  src/solver/solver_HQP_eiquadprog.cpp
  src/solver/solver_HQP_eiquadprog_fast.cpp
  src/solver/util.cpp

  src/formulation/inverse_dynamics_formulation_base.cpp
  src/formulation/inverse_dynamics_formulation_vel.cpp

)


ADD_SOURCE_GROUP(${PROJECT_NAME}_HEADERS)
ADD_SOURCE_GROUP(${PROJECT_NAME}_SOURCES)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS} )
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} SYSTEM PUBLIC $<INSTALL_INTERFACE:include> ${EIGEN3_INCLUDE_DIR} )

# exe for example
add_executable(${PROJECT_NAME}_test unittest/hqp_test.cpp)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES} eiquadprog::eiquadprog ${pinocchio_LIBRARIES} ${kimm_trajectory_smoother_LIBRARIES})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME}_test SYSTEM PUBLIC $<INSTALL_INTERFACE:include> ${EIGEN3_INCLUDE_DIR} ${PINOCCHIO_INCLUDE_DIR} ${eiquadprog_INCLUDE_DIRS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

