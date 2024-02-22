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

# generate_dynamic_reconfigure_options(cfg/PredictiveController_${CONFIGURATION_TO_USE}.cfg)
# message(WARNING "Predictive configuration: cfg/PredictiveController_${SYSTEM_TO_USE}.cfg")
catkin_package(
  CATKIN_DEPENDS ${DEPENDENCIES}
  INCLUDE_DIRS include include/${PROJECT_NAME}

  # LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME} INTERFACE)
# add_dependencies(${PROJECT_NAME} INTERFACE ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_definitions(-DROS${MPC_PLANNER_ROS})

# ## Test Case ####
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(yaml-cpp REQUIRED)
# find_package(mpc-planner-types REQUIRED)

# if(MPC_PLANNER_ROS STREQUAL "2")
# # message("MPC_PLANNER_ROS 2")
# find_package(rclcpp REQUIRED)
# find_package(ros_tools REQUIRED)
# list(APPEND DEPENDENCIES
# rclcpp
# ros_tools
# )
# endif()

# # # BUILD ###
# include_directories(
# include
# )

# set(ENV{MPC_PLANNER_ROS} ${MPC_PLANNER_ROS})

# add_library(${PROJECT_NAME}
# src/visuals.cpp
# src/spline.cpp
# src/third_party/tkspline.cpp
# )
# target_include_directories(${PROJECT_NAME} PUBLIC
# "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
# "$<INSTALL_INTERFACE:include>"
# )
# target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
# ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

# if(BUILD_TESTING)

# endif()

# install(
# TARGETS ${PROJECT_NAME}
# EXPORT export_${PROJECT_NAME}
# LIBRARY DESTINATION lib
# ARCHIVE DESTINATION lib
# RUNTIME DESTINATION bin
# )

# install(DIRECTORY include/${PROJECT_NAME}
# DESTINATION include/)

# install(DIRECTORY include/third_party/
# DESTINATION include/third_party/)

# # install(DIRECTORY Solver/include/
# # DESTINATION include/${PROJECT_NAME}/)
# # ament_export_include_directories(include/${PROJECT_NAME})
# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_dependencies(${DEPENDENCIES})

# ament_package()