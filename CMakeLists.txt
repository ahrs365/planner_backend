cmake_minimum_required(VERSION 3.20)

project(planner_backend VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build options
option(BUILD_DWA_SERVER "Build DWA planner server" ON)
option(BUILD_DDR_OPT_SERVER "Build DDR optimization server" ON)
option(BUILD_TMPC_SERVER "Build TMPC planner server" ON)
option(BUILD_BSPLINE_SERVER "Build Bspline planner server" ON)

# CMake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Third party paths
set(THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
set(ACADOS_LIB_DIR "${THIRD_PARTY_DIR}/acados/lib")

# RPATH settings for acados
set(CMAKE_BUILD_RPATH "${ACADOS_LIB_DIR}")
set(CMAKE_INSTALL_RPATH "${ACADOS_LIB_DIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")

# yaml-cpp
if(NOT TARGET yaml-cpp)
  set(YAML_CPP_BUILD_TESTS OFF CACHE INTERNAL "")
  set(YAML_CPP_BUILD_SHARED_LIBS ON CACHE INTERNAL "")
  set(CMAKE_POSITION_INDEPENDENT_CODE ON CACHE INTERNAL "")
  add_subdirectory(${THIRD_PARTY_DIR}/yaml-cpp third_party_yaml_cpp)
endif()

# TMPC libraries (required for tmpc_server)
if(BUILD_TMPC_SERVER)
  set(BUILD_MAIN_EXECUTABLE OFF CACHE BOOL "" FORCE)
  add_subdirectory(src/tmpc)
endif()

# Planner servers
add_subdirectory(src/servers)

message(STATUS "========================================")
message(STATUS "Planner Backend Build Configuration")
message(STATUS "========================================")
message(STATUS "DWA Server:      ${BUILD_DWA_SERVER}")
message(STATUS "DDR-OPT Server:  ${BUILD_DDR_OPT_SERVER}")
message(STATUS "TMPC Server:     ${BUILD_TMPC_SERVER}")
message(STATUS "Bspline Server:  ${BUILD_BSPLINE_SERVER}")
message(STATUS "========================================")

