cmake_minimum_required(VERSION 2.8.3)
project(mpc_planner_msgs)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  std_msgs
  geometry_msgs
)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES Gaussian.msg ObstacleGMM.msg ObstacleArray.msg
)

generate_messages(DEPENDENCIES nav_msgs std_msgs geometry_msgs)

catkin_package(
  DEPENDS nav_msgs std_msgs geometry_msgs
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
