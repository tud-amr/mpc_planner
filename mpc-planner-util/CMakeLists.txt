cmake_minimum_required(VERSION 3.8)
project(mpc-planner-util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(MPC_PLANNER_ROS "1")

# General requirements
set(DEPENDENCIES
  mpc-planner-types
  ros_tools
  roscpp
)

find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
  REQUIRED
  Eigen3
)

add_definitions(${EIGEN_DEFINITIONS})

# generate_dynamic_reconfigure_options(cfg/PredictiveController_${CONFIGURATION_TO_USE}.cfg)
# message(WARNING "Predictive configuration: cfg/PredictiveController_${SYSTEM_TO_USE}.cfg")
catkin_package(
  CATKIN_DEPENDS ${DEPENDENCIES}
  INCLUDE_DIRS include include/${PROJECT_NAME}
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/spline.cpp
  src/visuals.cpp
  src/third_party/tkspline.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_definitions(-DROS${MPC_PLANNER_ROS})

# ## Test Case ####
catkin_add_gtest(${PROJECT_NAME}-test test/test_spline.cpp)
target_link_libraries(${PROJECT_NAME}-test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)