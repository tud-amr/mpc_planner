cmake_minimum_required(VERSION 3.8)
project(mpc-planner-dingo)

add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  roscpp
  mpc-planner
  mpc-planner-solver
  mpc-planner-util
  mpc_planner_types
  mpc_planner_msgs
  ros_tools
  nav_msgs
)

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS ${DEPENDENCIES}
  INCLUDE_DIRS include include/${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dingo_planner src/ros1_planner.cpp)
add_dependencies(dingo_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dingo_planner
  ${catkin_LIBRARIES}
  yaml-cpp
)
install(
  TARGETS dingo_planner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)