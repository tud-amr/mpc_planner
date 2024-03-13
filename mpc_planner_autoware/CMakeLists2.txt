cmake_minimum_required(VERSION 3.8)
project(mpc_planner_autoware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(WARNING "Compiling without OPENMP (OPENMP not found)")
endif()

set(DEPENDENCIES
  mpc-planner
  mpc-planner-solver
  mpc-planner-modules
  mpc-planner-util
  mpc_planner_types
  mpc_planner_msgs
  ros_tools
  rclcpp
  nav_msgs
  std_msgs
  std_srvs
  autoware_auto_planning_msgs
  autoware_auto_vehicle_msgs
  autoware_auto_perception_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mpc-planner REQUIRED)
find_package(mpc-planner-solver REQUIRED)
find_package(mpc-planner-modules REQUIRED)
find_package(mpc-planner-util REQUIRED)
find_package(mpc_planner_types REQUIRED)
find_package(mpc_planner_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(ros_tools REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(autoware_auto_planning_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)

include_directories(
  include
)

add_executable(autoware_planner src/ros2_autoware.cpp)
target_include_directories(autoware_planner PUBLIC
"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
"$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_compile_features(autoware_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(autoware_planner ${DEPENDENCIES})


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

install(TARGETS autoware_planner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
  
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${DEPENDENCIES})
ament_package()
