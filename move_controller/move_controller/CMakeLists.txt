cmake_minimum_required(VERSION 2.8.3)
project(move_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmd_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(
move_controller src/move_controller.cpp src/JHPWMPCA9685.cpp
)

target_link_libraries(move_controller
   ${catkin_LIBRARIES}
 )


