cmake_minimum_required(VERSION 3.8)
project(mpc_planner_modules)

add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(OpenMP)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(WARNING "Compiling without OPENMP")
endif()

set(DEPENDENCIES
  mpc_planner_types
  mpc_planner_util
  mpc_planner_solver
  ros_tools
)

include(modules.cmake)
list(APPEND DEPENDENCIES ${MODULE_DEPENDENCIES})

find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS ${DEPENDENCIES}
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

# # BUILD ###
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  ${MODULE_SOURCES}
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_definitions(-DMPC_PLANNER_ROS)
add_definitions(-DDECOMP_OLD)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)