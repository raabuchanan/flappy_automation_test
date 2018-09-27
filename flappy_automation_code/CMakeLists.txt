cmake_minimum_required(VERSION 2.8.3)
project(flappy_automation_code)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_library
  src/FlappyNavigation.cpp
  src/PipeGapTracker.cpp
)

target_link_libraries(${PROJECT_NAME}_library
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/flappy_automation_code.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_library
)
