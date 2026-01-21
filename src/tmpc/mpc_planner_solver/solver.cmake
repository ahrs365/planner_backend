add_definitions(-DACADOS_SOLVER)
set(ACADOS_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../..//third_party/acados")
set(ACADOS_LIB_DIR "${ACADOS_SOURCE_DIR}/lib")
find_library(acados_LIBRARY libacados.so PATHS ${ACADOS_LIB_DIR} NO_DEFAULT_PATH)
find_library(blasfeo_LIBRARY libblasfeo.so PATHS ${ACADOS_LIB_DIR} NO_DEFAULT_PATH)
find_library(hpipm_LIBRARY libhpipm.so PATHS ${ACADOS_LIB_DIR} NO_DEFAULT_PATH)
set(acados_include_path ${ACADOS_SOURCE_DIR}/include)
# Print acados_include_path
set(solver_LIBRARIES
    ${PROJECT_SOURCE_DIR}/acados/Solver/libacados_ocp_solver_Solver.so # Generated files
    ${acados_LIBRARY}
    ${blasfeo_LIBRARY}
    ${hpipm_LIBRARY}
)
set(solver_INCLUDE_DIRS
    acados/Solver # Generated files
    acados # Generated files
    ${acados_include_path}
    ${acados_include_path}/blasfeo/include
    ${acados_include_path}/hpipm/include
)
set(solver_SOURCES
    src/acados_solver_interface.cpp
)
