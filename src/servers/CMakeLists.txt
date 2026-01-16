set(THIRD_PARTY_DIR "${CMAKE_SOURCE_DIR}/third_party")
set(SRC_DIR "${CMAKE_SOURCE_DIR}/src")

set(WEBSOCKETPP_DIR "${THIRD_PARTY_DIR}/websocketpp")
set(ASIO_DIR "${THIRD_PARTY_DIR}/asio")
set(JSON_DIR "${THIRD_PARTY_DIR}/json")
set(YAML_CPP_DIR "${THIRD_PARTY_DIR}/yaml-cpp")
set(EIGEN_DIR "${THIRD_PARTY_DIR}/eigen")
set(ACADOS_LIB_DIR "${THIRD_PARTY_DIR}/acados/lib")

# Verify dependencies
if(NOT EXISTS "${WEBSOCKETPP_DIR}/websocketpp")
  message(FATAL_ERROR "websocketpp not found at ${WEBSOCKETPP_DIR}")
endif()
if(NOT EXISTS "${ASIO_DIR}/asio/include")
  message(FATAL_ERROR "asio not found at ${ASIO_DIR}")
endif()
if(NOT EXISTS "${JSON_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "json not found at ${JSON_DIR}")
endif()
if(NOT EXISTS "${EIGEN_DIR}/Eigen")
  message(FATAL_ERROR "eigen not found at ${EIGEN_DIR}")
endif()

# nlohmann_json
set(JSON_BuildTests OFF CACHE INTERNAL "")
set(JSON_Install OFF CACHE INTERNAL "")
add_subdirectory(${JSON_DIR} third_party_json)

find_package(Threads REQUIRED)

# Planner server template
function(add_planner_server target)
  cmake_parse_arguments(APS "" "" "PLANNER_SOURCES;SERVER_SOURCES;EXTRA_LIBS;EXTRA_INCLUDES" ${ARGN})
  add_executable(${target} ${APS_PLANNER_SOURCES} ${APS_SERVER_SOURCES})
  target_include_directories(${target} PRIVATE
    ${WEBSOCKETPP_DIR}
    ${ASIO_DIR}/asio/include
    ${EIGEN_DIR}
    ${SRC_DIR}
    ${APS_EXTRA_INCLUDES}
  )
  target_compile_definitions(${target} PRIVATE ASIO_STANDALONE)
  target_link_libraries(${target} PRIVATE nlohmann_json::nlohmann_json Threads::Threads ${APS_EXTRA_LIBS})
  target_compile_features(${target} PRIVATE cxx_std_17)
endfunction()

# DWA Server
if(BUILD_DWA_SERVER)
  add_planner_server(dwa_server
    PLANNER_SOURCES
      ${SRC_DIR}/dwa/dwa_planner.cpp
    SERVER_SOURCES
      ${CMAKE_CURRENT_SOURCE_DIR}/dwa_server.cpp
  )
endif()

# DDR-OPT Server
if(BUILD_DDR_OPT_SERVER)
  add_planner_server(ddr_opt_server
    PLANNER_SOURCES
      ${SRC_DIR}/ddr-opt/esdf/esdf_map.cpp
      ${SRC_DIR}/ddr-opt/path/jps_planner.cpp
      ${SRC_DIR}/ddr-opt/path/graph_search.cpp
      ${SRC_DIR}/ddr-opt/opt/optimizer.cpp
    SERVER_SOURCES
      ${CMAKE_CURRENT_SOURCE_DIR}/ddr_opt_server.cpp
    EXTRA_LIBS
      yaml-cpp
    EXTRA_INCLUDES
      ${SRC_DIR}/ddr-opt
      ${SRC_DIR}/ddr-opt/esdf
      ${SRC_DIR}/ddr-opt/path
      ${SRC_DIR}/ddr-opt/opt
  )
  target_compile_definitions(ddr_opt_server PRIVATE
    DDR_OPT_CONFIG_PATH="${SRC_DIR}/ddr-opt/opt/global_planning3ms.yaml"
  )
endif()

# TMPC Server
if(BUILD_TMPC_SERVER)
  add_planner_server(tmpc_server
    SERVER_SOURCES
      ${CMAKE_CURRENT_SOURCE_DIR}/tmpc_server.cpp
    EXTRA_LIBS
      mpc_planner
      mpc_planner_modules
      mpc_planner_solver
      mpc_planner_util
      mpc_planner_types
      guidance_planner
      guidance_planner_homotopy
      ros_tools_no_ros
      yaml-cpp
    EXTRA_INCLUDES
      ${SRC_DIR}/tmpc/mpc_planner/include
      ${SRC_DIR}/tmpc/mpc_planner_modules/include
      ${SRC_DIR}/tmpc/mpc_planner_solver/include
      ${SRC_DIR}/tmpc/mpc_planner_types/include
      ${SRC_DIR}/tmpc/mpc_planner_util/include
      ${SRC_DIR}/tmpc/guidance_planner/include
      ${SRC_DIR}/tmpc/ros_tools_no_ros/include
      ${SRC_DIR}/tmpc/DecompUtil/include
  )
  target_compile_definitions(tmpc_server PRIVATE
    TMPC_CONFIG_PATH="${SRC_DIR}/tmpc/mpc_planner_jackalsimulator/config/settings.yaml"
  )
  set_target_properties(tmpc_server PROPERTIES
    BUILD_RPATH "${ACADOS_LIB_DIR}"
    INSTALL_RPATH "${ACADOS_LIB_DIR}"
  )
endif()
