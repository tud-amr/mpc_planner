cmake_minimum_required(VERSION 3.8)
project(mpc-planner-util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(MPC_PLANNER_ROS "2")

# General requirements
set(DEPENDENCIES
  yaml-cpp
  mpc-planner-types
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(mpc-planner-types REQUIRED)

if(MPC_PLANNER_ROS STREQUAL "2")
  # message("MPC_PLANNER_ROS 2")
  find_package(rclcpp REQUIRED)
  find_package(ros_tools REQUIRED)
  list(APPEND DEPENDENCIES 
    rclcpp
    ros_tools
  )
endif()

# # BUILD ###
include_directories(
  include
)

set(ENV{MPC_PLANNER_ROS} ${MPC_PLANNER_ROS})

add_library(${PROJECT_NAME}
  src/visuals.cpp
  src/spline.cpp
  src/third_party/tkspline.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})


if(BUILD_TESTING)

endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/)

install(DIRECTORY include/third_party/
  DESTINATION include/third_party/)

  # install(DIRECTORY Solver/include/
  # DESTINATION include/${PROJECT_NAME}/)
# ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

ament_package()