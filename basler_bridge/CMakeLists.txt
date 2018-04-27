cmake_minimum_required(VERSION 2.8.3)
project(basler_bridge)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif (COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else ()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()

find_package(OpenMP)
if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Basler REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  dynamic_reconfigure
  cv_bridge
  message_generation
  image_transport
  image_geometry
  params_lib)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/basler_tof_bridge.cfg
  cfg/basler_rgb_bridge.cfg)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES basler_tof_bridge basler_rgb_bridge
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs dynamic_reconfigure cv_bridge message_runtime image_transport image_geometry params_lib
  DEPENDS OpenCV PCL Basler)

###########
## Build ##
###########

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${BASLER_TOF_INCLUDE_DIR}
  ${BASLER_RGB_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS} ${BASLER_TOF_LIBRARIES_DIR} ${BASLER_RGB_LIBRARIES_DIR})
add_definitions(${PCL_DEFINITIONS})

file(GLOB basler_bridge_SRC
  "data/*/*.yaml"
  "data/*/*.pfs"
  "launch/*"
  "package.xml"
  "nodelet_plugins.xml"
  "cfg/*"
  "scripts/*")

file(GLOB basler_tof_bridge_SRC
  "include/BaslerTOF.h"
  "src/basler_tof_bridge.cpp")
add_library(basler_tof_bridge ${basler_tof_bridge_SRC} ${basler_bridge_SRC})
add_dependencies(basler_tof_bridge ${PROJECT_NAME}_gencfg)
target_link_libraries(
  basler_tof_bridge
  ${params_lib_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${BASLER_TOF_LIBRARIES})

file(GLOB basler_rgb_bridge_SRC
  "include/BaslerRGB.h"
  "src/basler_rgb_bridge.cpp")
add_library(basler_rgb_bridge ${basler_rgb_bridge_SRC} ${basler_bridge_SRC})
add_dependencies(basler_rgb_bridge ${PROJECT_NAME}_gencfg)
target_link_libraries(
  basler_rgb_bridge
  ${params_lib_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${BASLER_RGB_LIBRARIES})

#############
## Install ##
#############

install(TARGETS basler_tof_bridge basler_rgb_bridge
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/robot_calib_auto.py
  scripts/robot_calib_lib.py
  scripts/robot_calib_static.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY data launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
