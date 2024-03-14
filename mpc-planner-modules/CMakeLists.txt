cmake_minimum_required(VERSION 3.8)
project(mpc-planner-modules)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

option(USE_ROS2 "Use ROS2 CMake" ON)

find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(WARNING "Compiling without OPENMP (OPENMP not found)")
endif()

# General requirements
set(DEPENDENCIES
  ros_tools
  mpc_planner_types
  mpc-planner-util
  mpc-planner-solver
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(pkg IN LISTS DEPENDENCIES)
  find_package(${pkg} REQUIRED)
endforeach()

include(modules.cmake)
list(APPEND DEPENDENCIES ${MODULE_DEPENDENCIES})

## BUILD ###
include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  ${MODULE_SOURCES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
