cmake_minimum_required(VERSION 3.5)
project(mpc_planner_msgs)

set(DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

foreach(pkg IN LISTS DEPENDENCIES)
  find_package(${pkg} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleArray.msg"
  "msg/Gaussian.msg"
  "msg/ObstacleGMM.msg"
  DEPENDENCIES ${DEPENDENCIES}
)

# Generate messages in the 'msg' folder
ament_export_dependencies(${DEPENDENCIES})
ament_export_dependencies(rosidl_default_runtime)
ament_package()