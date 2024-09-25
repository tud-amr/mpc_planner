cmake_minimum_required(VERSION 3.8)
project(mpc_planner_jackalsimulator)

set(robot_name "jackalsimulator")

add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  roscpp
  mpc_planner
  mpc_planner_msgs
  mpc_planner_modules
  mpc_planner_solver
  mpc_planner_util
  mpc_planner_types
  ros_tools
  nav_msgs
  dynamic_reconfigure
  tf2_ros
)

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
)

generate_dynamic_reconfigure_options(cfg/${robot_name}.cfg)

catkin_package(
  CATKIN_DEPENDS ${DEPENDENCIES}
  INCLUDE_DIRS include/${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${robot_name}_planner src/ros1_${robot_name}.cpp)
add_dependencies(${robot_name}_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${robot_name}_planner ${catkin_LIBRARIES})

add_definitions(-DMPC_PLANNER_ROS)

install(
  TARGETS ${robot_name}_planner
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)