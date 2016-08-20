cmake_minimum_required(VERSION 2.8.3)
project(depth_object_detector)

# Set up the ROS Catkin package settings.
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  std_msgs
  sensor_msgs
  image_transport
  roscpp
  cmake_modules
  message_generation
  geometry_msgs
  nodelet
  depth_image_proc
)

generate_messages(DEPENDENCIES
    std_msgs
    geometry_msgs
)

# Set up the ROS Catkin package settings
catkin_package(
  CATKIN_DEPENDS std_msgs
                 sensor_msgs
                 image_transport
                 roscpp
                 geometry_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}    src/nodelets/nodelet.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
