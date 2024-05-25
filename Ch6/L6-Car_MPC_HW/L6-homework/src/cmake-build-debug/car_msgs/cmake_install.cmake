# Install script for directory: /home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/car_msgs

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
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/car_msgs/msg" TYPE FILE FILES "/home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/car_msgs/msg/CarCmd.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/car_msgs/cmake" TYPE FILE FILES "/home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/cmake-build-debug/car_msgs/catkin_generated/installspace/car_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/cmake-build-debug/devel/include/car_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/cmake-build-debug/devel/share/roseus/ros/car_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/cmake-build-debug/devel/share/common-lisp/ros/car_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/cmake-build-debug/devel/share/gennodejs/ros/car_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/cmake-build-debug/devel/lib/python3/dist-packages/car_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/cmake-build-debug/devel/lib/python3/dist-packages/car_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/cmake-build-debug/car_msgs/catkin_generated/installspace/car_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/car_msgs/cmake" TYPE FILE FILES "/home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/cmake-build-debug/car_msgs/catkin_generated/installspace/car_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/car_msgs/cmake" TYPE FILE FILES
    "/home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/cmake-build-debug/car_msgs/catkin_generated/installspace/car_msgsConfig.cmake"
    "/home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/cmake-build-debug/car_msgs/catkin_generated/installspace/car_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/car_msgs" TYPE FILE FILES "/home/wrk/File/Other_file/SLAM/My_course/Motion_Planning/MP_repo/Ch6/L6-Car_MPC_HW/L6-homework/src/car_msgs/package.xml")
endif()

