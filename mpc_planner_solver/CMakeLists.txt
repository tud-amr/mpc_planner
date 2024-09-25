cmake_minimum_required(VERSION 3.8)
project(mpc_planner_solver)

add_compile_options(-std=c++17)

set(DEPENDENCIES
  mpc_planner_util
)

find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
)

include(solver.cmake)

message(WARNING ${solver_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS ${DEPENDENCIES}
  INCLUDE_DIRS
  include
  include/${PROJECT_NAME}
  ${solver_INCLUDE_DIRS}
  LIBRARIES
  ${PROJECT_NAME}
  ${solver_LIBRARIES}
  CFG_EXTRAS ${PROJECT_SOURCE_DIR}/mpc_planner_solver-extras.cmake # Exports definitions
)

include_directories(
  include
  ${solver_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/mpc_planner_parameters.cpp
  src/state.cpp
  ${solver_SOURCES}
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${solver_LIBRARIES}
)

add_definitions(-DMPC_PLANNER_ROS)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ${solver_INCLUDE_DIRS}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../
  PATTERN ".svn" EXCLUDE
)
