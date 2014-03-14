cmake_minimum_required(VERSION 2.8.3)
project(block_grasp_generator)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread system)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED
  roscpp
  std_msgs
  visualization_msgs
  eigen_conversions
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_robot_interaction
  moveit_core
  tf
  tf_conversions
  std_msgs
  trajectory_msgs
  message_generation
  geometry_msgs
  actionlib
  moveit_msgs
)

add_action_files(DIRECTORY action FILES
  GenerateBlockGrasps.action
)

generate_messages(DEPENDENCIES
  geometry_msgs actionlib_msgs std_msgs moveit_msgs)

# Visualization Tools Library
add_library(${PROJECT_NAME}_visualization_tools
  src/visualization_tools.cpp
)
target_link_libraries(${PROJECT_NAME}_visualization_tools 
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Grasp Generator Library
add_library(${PROJECT_NAME}
  src/block_grasp_generator.cpp
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME}_visualization_tools
)

# Grasp Filter Library
add_library(${PROJECT_NAME}_filter
  src/grasp_filter.cpp
)
target_link_libraries(${PROJECT_NAME}_filter 
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME}_visualization_tools
)

# Catkin
catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
    ${PROJECT_NAME}_filter
    ${PROJECT_NAME}_visualization_tools
  CATKIN_DEPENDS
    roscpp
    std_msgs
    trajectory_msgs
    visualization_msgs 
    eigen_conversions 
    moveit_ros_planning 
    moveit_ros_planning_interface
    moveit_ros_robot_interaction
    moveit_core
    tf
    tf_conversions
    std_msgs
    message_runtime
  INCLUDE_DIRS include
)

# Enable gdb to show line numbers
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g")
#set(CMAKE_BUILD_TYPE Debug)

## Build 
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

# Test executable
add_executable(block_grasp_generator_server src/block_grasp_generator_server.cpp)
target_link_libraries(block_grasp_generator_server
  ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Test executable
add_executable(grasp_filter_test src/grasp_filter_test.cpp)
target_link_libraries(grasp_filter_test 
  ${catkin_LIBRARIES} ${PROJECT_NAME} ${PROJECT_NAME}_filter ${PROJECT_NAME}_visualization_tools
)

# Test executable
add_executable(visualization_tools_test src/visualization_tools_test.cpp)
target_link_libraries(visualization_tools_test 
  ${catkin_LIBRARIES} ${PROJECT_NAME} ${PROJECT_NAME}_visualization_tools
)

# Test executable
add_executable(block_grasp_generator_test src/block_grasp_generator_test.cpp)
target_link_libraries(block_grasp_generator_test 
  ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)



