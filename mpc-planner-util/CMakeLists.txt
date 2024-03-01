cmake_minimum_required(VERSION 3.8)
project(mpc-planner-util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(MPC_PLANNER_ROS "1")

# General requirements
set(DEPENDENCIES
  mpc-planner-types
  ros_planner_utils
  roscpp
)

find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS ${DEPENDENCIES}
  INCLUDE_DIRS include include/${PROJECT_NAME}
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/data_visualization.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

# add_definitions(-DROS${MPC_PLANNER_ROS})

# ## Test Case ####
# catkin_add_gtest(${PROJECT_NAME}-test test/test_spline.cpp)
# target_link_libraries(${PROJECT_NAME}-test
# ${PROJECT_NAME}
# ${catkin_LIBRARIES}
# yaml-cpp
# )
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)