cmake_minimum_required(VERSION 3.8)
project(mpc_planner_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

set(DEPENDENCIES
  mpc_planner_util
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(pkg IN LISTS DEPENDENCIES)
  find_package(${pkg} REQUIRED)
endforeach()

include_directories(
  include
  Solver/include
)

set(SOLVER_LIBRARY ${PROJECT_SOURCE_DIR}/Solver/lib/libSolver.so)
add_library(${PROJECT_NAME} SHARED
  Solver/Solver_interface.c
  Solver/Solver_model.c
  src/solver_interface.cpp
  src/state.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} ${SOLVER_LIBRARY})

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/test_solver.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    ${DEPENDENCIES}
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

install(
  TARGETS 
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/)

install(DIRECTORY Solver/include/
  DESTINATION include/${PROJECT_NAME}/)

install(
  DIRECTORY Solver/lib/
  DESTINATION lib
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
