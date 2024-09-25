cmake_minimum_required(VERSION 3.8)
project(mpc_planner_rosnavigation)

add_compile_options(-std=c++17)

set(DEPENDENCIES
  base_local_planner
  costmap_2d
  costmap_converter
  roscpp
  mpc_planner
  mpc_planner_msgs
  mpc_planner_modules
  mpc_planner_solver
  mpc_planner_util
  mpc_planner_types
  nav_core
  ros_tools
  nav_msgs
  dynamic_reconfigure
  tf2_ros
)

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
)

generate_dynamic_reconfigure_options(cfg/rosnavigation.cfg)

catkin_package(
  CATKIN_DEPENDS ${DEPENDENCIES}
  INCLUDE_DIRS include/${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/goal_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(${PROJECT_NAME}
  src/ros1_rosnavigation.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_definitions(-DMPC_PLANNER_ROS)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
  mpc_planner_rosnavigation_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)