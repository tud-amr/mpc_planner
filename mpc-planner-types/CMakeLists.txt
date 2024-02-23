cmake_minimum_required(VERSION 3.8)
project(mpc-planner-types)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# General requirements
set(DEPENDENCIES
)

find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS ${DEPENDENCIES}
  INCLUDE_DIRS include
)

include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# add_definitions(-DROS${MPC_PLANNER_ROS})

# ## Test Case ####
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)