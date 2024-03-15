cmake_minimum_required(VERSION 3.8)
project(mpc_planner-jackal)

add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  roscpp
  mpc_planner
  mpc_planner_solver
  mpc_planner_util
  mpc_planner_types
  mpc_planner_msgs
  ros_tools
  nav_msgs
  derived_object_msgs
)

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS ${DEPENDENCIES}
  INCLUDE_DIRS include include/${PROJECT_NAME}

  # LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(jackal_planner src/ros1_jackal.cpp)
add_dependencies(jackal_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(jackal_planner
  ${catkin_LIBRARIES}
  yaml-cpp
)
install(
  TARGETS jackal_planner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)