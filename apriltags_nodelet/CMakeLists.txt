cmake_minimum_required(VERSION 2.8.3)
project(apriltags_nodelet)

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
)

find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

# Import the yaml-cpp libraries.
include(FindPkgConfig)
pkg_check_modules(Yaml REQUIRED yaml-cpp)

add_message_files(DIRECTORY msg FILES
    AprilTagDetection.msg
    AprilTagDetections.msg
)

generate_messages(DEPENDENCIES
    std_msgs
    geometry_msgs
)

# Set up the ROS Catkin package settings
catkin_package(
  CATKIN_DEPENDS cv_bridge
                 std_msgs
                 sensor_msgs
                 image_transport
                 roscpp
                 geometry_msgs
)

# CGAL requires that -frounding-math be set.
add_definitions(-frounding-math)

# Download the external Swatbotics repository.
include(ExternalProject)
ExternalProject_Add(apriltags_swatbotics_EXTERNAL
   # GIT_REPOSITORY https://github.com/personalrobotics/apriltags-cpp.git
    URL /home/ubuntu/apriltags-cpp/
    INSTALL_COMMAND ""
    BUILD_COMMAND "make" # I don't know why this works...
    CMAKE_ARGS -DCMAKE_CXX_FLAGS=-frounding-math -DBUILD_SHARED_LIBS:BOOL=ON
)

# Recover project paths for additional settings.
ExternalProject_Get_Property(apriltags_swatbotics_EXTERNAL
  SOURCE_DIR BINARY_DIR INSTALL_DIR)
set(apriltags_swatbotics_INCLUDE_DIRS
  "${SOURCE_DIR}"
)
set(apriltags_swatbotics_LIBRARIES
  "${BINARY_DIR}/libapriltags.so"
)

# Tell CMake that the external project generated a library so we
# can add dependencies here instead of later.
add_library(apriltags_swatbotics UNKNOWN IMPORTED)
set_property(TARGET apriltags_swatbotics
  PROPERTY IMPORTED_LOCATION
  ${apriltags_swatbotics_LIBRARIES}
)
add_dependencies(apriltags_swatbotics apriltags_swatbotics_EXTERNAL)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${Yaml_INCLUDE_DIRS}
    ${apriltags_swatbotics_INCLUDE_DIRS}
)

add_library(apriltags_nodelet    src/nodelets/nodelet.cpp )
target_link_libraries(apriltags_nodelet ${catkin_LIBRARIES})
target_link_libraries(apriltags_nodelet ${Eigen_LIBRARIES})
target_link_libraries(apriltags_nodelet ${OpenCV_LIBRARIES})
target_link_libraries(apriltags_nodelet ${Yaml_LIBRARIES})
target_link_libraries(apriltags_nodelet apriltags_swatbotics)
add_dependencies(apriltags_nodelet ${PROJECT_NAME}_gencpp)


add_executable(apriltags_exe src/main.cpp)
target_link_libraries(apriltags_exe ${catkin_LIBRARIES})
target_link_libraries(apriltags_exe ${Eigen_LIBRARIES})
target_link_libraries(apriltags_exe ${OpenCV_LIBRARIES})
target_link_libraries(apriltags_exe ${Yaml_LIBRARIES})
target_link_libraries(apriltags_exe apriltags_swatbotics)
add_dependencies(apriltags_exe apriltags_nodelet_gencpp)


install(TARGETS apriltags_exe
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)

install(FILES ${apriltags_swatbotics_LIBRARIES}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

