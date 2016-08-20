cmake_minimum_required(VERSION 2.8.3)
project(gpu_robot_vision)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  image_geometry
  OpenCV
  sensor_msgs
  std_msgs
  roscpp
  nodelet
  cmake_modules
)

find_package(Eigen REQUIRED)

# CGAL requires that -frounding-math be set.
add_definitions(-frounding-math)

catkin_package(
  CATKIN_DEPENDS image_geometry roscpp sensor_msgs
  DEPENDS OpenCV
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(SYSTEM apriltags-cpp ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
include_directories(include)

add_library(${PROJECT_NAME}     src/nodelets/nodelet.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(gpu_robot_vision_exe src/gpu_robot_vision.cpp)

target_link_libraries(gpu_robot_vision_exe
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
)
