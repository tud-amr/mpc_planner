add_definitions(-DACADOS_SOLVER)
string(REPLACE ":" ";" LIBRARY_DIRS $ENV{LD_LIBRARY_PATH})
find_library(acados_LIBRARY libacados.so PATHS ${LIBRARY_DIRS})
find_library(blasfeo_LIBRARY libblasfeo.so PATHS ${LIBRARY_DIRS})
find_library(hpipm_LIBRARY libhpipm.so PATHS ${LIBRARY_DIRS})
# Get the acados path from the LD_LIBRARY_PATH
get_filename_component(acados_path ${acados_LIBRARY} DIRECTORY)
set(acados_path ${acados_path}/..)
set(acados_include_path ${acados_path}/include)
# Print acados_include_path
set(solver_LIBRARIES
    ${PROJECT_SOURCE_DIR}/acados/test/libacados_ocp_solver_Solver.so # Generated files
    ${acados_LIBRARY}
    ${blasfeo_LIBRARY}
    ${hpipm_LIBRARY}
)
set(solver_INCLUDE_DIRS
    acados/test # Generated files
    ${acados_include_path}
    ${acados_include_path}/blasfeo/include
    ${acados_include_path}/hpipm/include
)
set(solver_SOURCES
    src/acados_solver_interface.cpp
)
