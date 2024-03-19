cmake_minimum_required(VERSION 3.8)
project(mpc_planner_jackal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(SYSTEM_TO_USE "Jackal")

set(DEPENDENCIES
  mpc_planner
  mpc_planner_solver
  mpc_planner_util
  mpc_planner_types
  ros_tools
  rclcpp
  nav_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mpc_planner REQUIRED)
find_package(mpc_planner_solver REQUIRED)
find_package(mpc_planner_util REQUIRED)
find_package(mpc_planner_types REQUIRED)

find_package(rclcpp REQUIRED)
find_package(ros_tools REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  include
)

# add_library(${PROJECT_NAME}
#   src/${SYSTEM_TO_USE}Solver.cpp
# )
# target_include_directories(${PROJECT_NAME}
#   PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
# )
# ament_target_dependencies(${PROJECT_NAME}
#   ${DEPENDENCIES}
# )
# target_link_libraries(${PROJECT_NAME}
#   ${PROJECT_SOURCE_DIR}/${SYSTEM_TO_USE}Solver/lib/lib${SYSTEM_TO_USE}Solver.so
#   ${YAML_CPP_LIBRARIES}
# )


add_executable(jackal_planner src/ros2_jackal_planner.cpp)
target_include_directories(jackal_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(jackal_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(jackal_planner ${DEPENDENCIES})
# target_link_libraries(jackal_planner)


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

install(TARGETS jackal_planner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
  
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${DEPENDENCIES})
ament_package()
