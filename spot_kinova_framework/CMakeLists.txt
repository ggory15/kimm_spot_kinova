cmake_minimum_required(VERSION 3.0.2)
project(spot_kinova_framework)


add_compile_options(-std=c++17 -Ofast)

INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/ide.cmake)
INCLUDE(cmake/apple.cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(KORTEX_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/../thirdparty/kortex_include")
set(KORTEX_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/../thirdparty/kortex_lib")

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
  kimm_spot_kinova
  spot_kinova_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(kimm_spot_kinova REQUIRED)
find_package(kimm_trajectory_smoother REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    geometry_msgs
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
    kimm_spot_kinova
    spot_kinova_msgs
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
  ${kimm_spot_kinova_INCLUDE_DIRS}
  ${kimm_trajectory_smoother_INCLUDE_DIRS}
)

SET(${PROJECT_NAME}_HEADERS
  include/spot_kinova_framework/controller/controller.hpp

  include/spot_kinova_framework/utilities/cxxopts.hpp
  include/spot_kinova_framework/utilities/kinova_utilities.hpp

  include/spot_kinova_framework/servers/action_server_base.hpp
  include/spot_kinova_framework/servers/joint_posture_action_server.hpp
  include/spot_kinova_framework/servers/se3_action_server.hpp
  include/spot_kinova_framework/servers/walk_action_server.hpp
  include/spot_kinova_framework/servers/body_posture_action_server.hpp
  include/spot_kinova_framework/servers/wholebody_action_server.hpp
)

SET(${PROJECT_NAME}_SOURCES
  src/controller/controller.cpp

  src/utilities/kinova_utilities.cpp
  
  src/servers/joint_posture_action_server.cpp
  src/servers/se3_action_server.cpp
  src/servers/walk_action_server.cpp
  src/servers/body_posture_action_server.cpp
  src/servers/wholebody_action_server.cpp
)


set(EXTRA_LIB ${EXTRA_LIB} ${KORTEX_LIBRARY_DIR}/release/libKortexApiCpp.a)
set(EXTRA_INC ${EXTRA_INC} ${KORTEX_INCLUDE_DIRS} ${KORTEX_INCLUDE_DIRS}/client ${KORTEX_INCLUDE_DIRS}/common ${KORTEX_INCLUDE_DIRS}/messages ${KORTEX_INCLUDE_DIRS}/client_stubs ${KORTEX_INCLUDE_DIRS}/google)

ADD_SOURCE_GROUP(${PROJECT_NAME}_HEADERS)
ADD_SOURCE_GROUP(${PROJECT_NAME}_SOURCES)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS} )
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} SYSTEM PUBLIC $<INSTALL_INTERFACE:include> ${EIGEN3_INCLUDE_DIR} ${EXTRA_INC} )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC ${kimm_spot_kinova_LIBRARIES} ${EXTRA_LIB} ${pinocchio_LIBRARIES} ${kimm_trajectory_smoother_LIBRARIES} eiquadprog::eiquadprog -lpthread)

# exe for example
add_executable(${PROJECT_NAME}_simulation src/simulation/simulation.cpp)
target_link_libraries(${PROJECT_NAME}_simulation ${PROJECT_NAME} ${catkin_LIBRARIES} -lpthread)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME}_simulation SYSTEM PUBLIC $<INSTALL_INTERFACE:include>)

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

