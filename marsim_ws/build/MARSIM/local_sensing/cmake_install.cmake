# Install script for directory: /home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws/src/MARSIM/local_sensing

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/local_sensing_node/msg" TYPE FILE FILES
    "/home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws/src/MARSIM/local_sensing/msg/DynamicObstacleState.msg"
    "/home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws/src/MARSIM/local_sensing/msg/DynamicObstacleStateArray.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/local_sensing_node/cmake" TYPE FILE FILES "/home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws/build/MARSIM/local_sensing/catkin_generated/installspace/local_sensing_node-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws/devel/include/local_sensing_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws/devel/share/roseus/ros/local_sensing_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws/devel/share/common-lisp/ros/local_sensing_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws/devel/share/gennodejs/ros/local_sensing_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws/devel/lib/python3/dist-packages/local_sensing_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws/devel/lib/python3/dist-packages/local_sensing_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws/build/MARSIM/local_sensing/catkin_generated/installspace/local_sensing_node.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/local_sensing_node/cmake" TYPE FILE FILES "/home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws/build/MARSIM/local_sensing/catkin_generated/installspace/local_sensing_node-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/local_sensing_node/cmake" TYPE FILE FILES
    "/home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws/build/MARSIM/local_sensing/catkin_generated/installspace/local_sensing_nodeConfig.cmake"
    "/home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws/build/MARSIM/local_sensing/catkin_generated/installspace/local_sensing_nodeConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/local_sensing_node" TYPE FILE FILES "/home/jjm/桌面/小样本学习的去中心集群协同跟踪/marsim_ws/src/MARSIM/local_sensing/package.xml")
endif()

