cmake_minimum_required(VERSION 2.8.3)
project(od_compressed_image_transport)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/compressed_publisher.cfg
  cfg/compressed_subscriber.cfg)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES compressed_publisher compressed_subscriber republisher
  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport
  DEPENDS OpenCV)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

file(GLOB od_SRC
  "cfg/*"
  "launch/*"
  "nodelet_plugins.xml"
  "src/basler_tof_bridge.cpp")

add_library(compressed_publisher "src/compressed_publisher.cpp" ${od_SRC})
add_dependencies(compressed_publisher ${PROJECT_NAME}_gencfg)
target_link_libraries(compressed_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_library(compressed_subscriber "src/compressed_subscriber.cpp" ${od_SRC})
add_dependencies(compressed_subscriber ${PROJECT_NAME}_gencfg)
target_link_libraries(compressed_subscriber
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_library(republisher "src/republisher.cpp" ${od_SRC})
target_link_libraries(republisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

#############
## Install ##
#############

install(TARGETS compressed_publisher compressed_subscriber republisher
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES compressed_plugins.xml nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})