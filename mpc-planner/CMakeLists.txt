cmake_minimum_required(VERSION 3.8)
project(mpc-planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# General requirements
set(DEPENDENCIES
  rclcpp
  mpc-planner-util
  mpc-planner-types
  mpc-planner-solver
  yaml-cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mpc-planner-util REQUIRED)
find_package(mpc-planner-types REQUIRED)
find_package(mpc-planner-solver REQUIRED)
find_package(yaml-cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


## BUILD ###
include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/planner.cpp
  # src/ros_planner.cpp

  # ${MODULE_SOURCES}
  src/mpc_base.cpp
  src/goal_module.cpp
  # src/modules_objectives/reference_path.cpp

  # src/interfaces/interface.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${PROJECT_NAME}
  ${DEPENDENCIES}
)
target_link_libraries(${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}
)

# TODO: Test node
# add_executable(test_node src/test_node.cpp)
# target_include_directories(test_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(test_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# target_link_libraries(test_node ${PROJECT_NAME})

# install(TARGETS test_node
#   DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
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

# ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
