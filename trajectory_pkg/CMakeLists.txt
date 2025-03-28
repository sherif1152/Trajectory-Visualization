cmake_minimum_required(VERSION 3.0.2)
project(trajectory_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
  message_generation
 
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SaveTrajectory.srv
)

## Generate added messages and services with dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime 
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Build trajectory_saver_node
add_executable(trajectory_saver_node src/trajectory_saver_node.cpp)
add_dependencies(trajectory_saver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_saver_node ${catkin_LIBRARIES} jsoncpp)

## Build trajectory_reader_node
add_executable(trajectory_reader_node src/trajectory_reader_node.cpp)
add_dependencies(trajectory_reader_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_reader_node ${catkin_LIBRARIES} jsoncpp)