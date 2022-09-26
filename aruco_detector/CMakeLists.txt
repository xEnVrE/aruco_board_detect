cmake_minimum_required(VERSION 3.16)

project(aruco_detector)

add_compile_options(-std=c++14)

# Packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  cv_bridge
  image_transport
  tf
)

find_package(OpenCV)

# Messages
add_message_files(
  FILES
  MarkerList.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

# Catkin configuration
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS geometry_msgs roscpp std_msgs
 DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Main executable configuration
add_executable(
  ${PROJECT_NAME}_node
  src/camera_parameters.cpp
  src/image_converter.cpp
  src/aruco_detector_node.cpp
  src/main.cpp
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
