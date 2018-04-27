cmake_minimum_required(VERSION 2.8.3)
project(depth_registration)

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
find_package(OpenCL REQUIRED)

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

set(DEPTH_REG_CPU ON)
set(DEPTH_REG_OPENCL ON)
if (DEPTH_REG_OPENCL)
  add_definitions(-DDEPTH_REG_OPENCL)
  add_definitions(-DREG_OPENCL_FILE="${CMAKE_CURRENT_SOURCE_DIR}/registration/depth_registration.cl")
endif ()
if (DEPTH_REG_CPU)
  add_definitions(-DDEPTH_REG_CPU)
endif ()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(cfg/depth_registration.cfg)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs dynamic_reconfigure cv_bridge message_runtime image_transport image_geometry params_lib
  DEPENDS OpenCV PCL OpenCL)

###########
## Build ##
###########

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/registration
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OPENCL_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB depth_registration_SRC
  "package.xml"
  "nodelet_plugins.xml"
  "src/depth_registration.cpp"
  "registration/*"
  "launch/*")
add_library(depth_registration ${depth_registration_SRC})
add_dependencies(depth_registration ${PROJECT_NAME}_gencfg)
target_link_libraries(depth_registration
  ${params_lib_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OPENCL_LIBRARIES})

#############
## Install ##
#############

install(TARGETS depth_registration
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY registration
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.cl")

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})