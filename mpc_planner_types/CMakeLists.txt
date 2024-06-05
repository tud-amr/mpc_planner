cmake_minimum_required(VERSION 3.8)
project(mpc_planner_types)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## DEPENDENCIES ##
set(DEPENDENCIES
  Eigen3
)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

## Build ##
include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/module_data.cpp
  src/data_types.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME})
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
