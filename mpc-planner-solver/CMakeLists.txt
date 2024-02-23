cmake_minimum_required(VERSION 3.8)
project(mpc-planner-solver)

add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

set(DEPENDENCIES
  mpc-planner-util
)

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS ${DEPENDENCIES}
  INCLUDE_DIRS include include/${PROJECT_NAME} Solver/include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  Solver/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/solver_interface.cpp
  src/state.cpp
  Solver/Solver_interface.c
  Solver/Solver_model.c
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/Solver/lib/libSolver.so
)

catkin_add_gtest(${PROJECT_NAME}-test test/test_solver.cpp)
target_link_libraries(${PROJECT_NAME}-test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
