if(EMSCRIPTEN)
  set(WASM_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/src")

  set(EMCC_LINK_FLAGS
    "-O3"
    "--bind"
    "-sWASM=1"
    "-sMODULARIZE=1"
    "-sEXPORT_ES6=1"
    "-sENVIRONMENT=web"
    "-sALLOW_MEMORY_GROWTH=1"
  )

  function(add_wasm_module target)
    add_executable(${target} ${ARGN})
    set_target_properties(${target} PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${WASM_OUTPUT_DIR}"
      OUTPUT_NAME "${target}"
    )
    target_link_options(${target} PRIVATE ${EMCC_LINK_FLAGS})
  endfunction()

  add_wasm_module(dwa
    "${CMAKE_CURRENT_LIST_DIR}/dwa/dwa_planner.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/dwa/dwa_wasm.cpp"
  )
else()
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
  set(ACADOS_LIB_DIR "${CMAKE_SOURCE_DIR}/third_party/acados/lib")
  set(CMAKE_BUILD_RPATH "${ACADOS_LIB_DIR}")
  set(CMAKE_INSTALL_RPATH "${ACADOS_LIB_DIR}")
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")
  if(NOT TARGET yaml-cpp)
    set(YAML_CPP_BUILD_TESTS OFF CACHE INTERNAL "")
    set(YAML_CPP_BUILD_SHARED_LIBS ON CACHE INTERNAL "")
    set(CMAKE_POSITION_INDEPENDENT_CODE ON CACHE INTERNAL "")
    add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/yaml-cpp third_party_yaml_cpp)
  endif()
  set(BUILD_MAIN_EXECUTABLE OFF CACHE BOOL "" FORCE)
  add_subdirectory(tmpc)
  add_subdirectory(native)
endif()
