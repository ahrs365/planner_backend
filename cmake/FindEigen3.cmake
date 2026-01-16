set(_EIGEN3_HINT "${CMAKE_SOURCE_DIR}/third_party/eigen")

if(EXISTS "${_EIGEN3_HINT}/Eigen/Core")
  set(EIGEN3_INCLUDE_DIR "${_EIGEN3_HINT}")
  set(EIGEN3_INCLUDE_DIRS "${_EIGEN3_HINT}")
  set(Eigen3_FOUND TRUE)
  if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${_EIGEN3_HINT}"
    )
  endif()
else()
  set(Eigen3_FOUND FALSE)
  message(FATAL_ERROR "Eigen3 not found. Expected in ${_EIGEN3_HINT}")
endif()
