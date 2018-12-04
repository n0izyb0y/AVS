cmake_minimum_required(VERSION 2.8.3)
project(master_station)

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
master_station src/master_station.cpp
)

target_link_libraries(master_station
   ${catkin_LIBRARIES}
 )


