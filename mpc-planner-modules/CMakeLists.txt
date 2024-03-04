cmake_minimum_required(VERSION 3.8)
project(mpc-planner-modules)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(OpenMP)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  message(WARNING "Compiling with OPENMP")
else()
  message(WARNING "Compiling without OPENMP")
endif()

# General requirements
set(DEPENDENCIES
  mpc_planner_types
  mpc-planner-util
  mpc-planner-solver
  ros_tools
  guidance_planner # TEMP
)

find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS ${DEPENDENCIES}
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include(modules.cmake)

# # BUILD ###
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${MODULE_SOURCES}
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)