# Install script for directory: /home/gao/workspace/pnc_project/web_robot/backend/src/tmpc

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/doc/mpc_planner" TYPE FILE FILES "/home/gao/workspace/pnc_project/web_robot/backend/src/tmpc/README.md")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/gao/workspace/pnc_project/web_robot/backend/build/src/tmpc/ros_tools_no_ros/cmake_install.cmake")
  include("/home/gao/workspace/pnc_project/web_robot/backend/build/src/tmpc/mpc_planner_types/cmake_install.cmake")
  include("/home/gao/workspace/pnc_project/web_robot/backend/build/src/tmpc/mpc_planner_util/cmake_install.cmake")
  include("/home/gao/workspace/pnc_project/web_robot/backend/build/src/tmpc/mpc_planner_solver/cmake_install.cmake")
  include("/home/gao/workspace/pnc_project/web_robot/backend/build/src/tmpc/guidance_planner/cmake_install.cmake")
  include("/home/gao/workspace/pnc_project/web_robot/backend/build/src/tmpc/DecompUtil/cmake_install.cmake")
  include("/home/gao/workspace/pnc_project/web_robot/backend/build/src/tmpc/mpc_planner_modules/cmake_install.cmake")
  include("/home/gao/workspace/pnc_project/web_robot/backend/build/src/tmpc/mpc_planner/cmake_install.cmake")

endif()

