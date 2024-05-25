# Install script for directory: /home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/mpc_car

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
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/cmake-build-debug/mpc_car/catkin_generated/installspace/mpc_car.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mpc_car/cmake" TYPE FILE FILES
    "/home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/cmake-build-debug/mpc_car/catkin_generated/installspace/mpc_carConfig.cmake"
    "/home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/cmake-build-debug/mpc_car/catkin_generated/installspace/mpc_carConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mpc_car" TYPE FILE FILES "/home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/mpc_car/package.xml")
endif()

