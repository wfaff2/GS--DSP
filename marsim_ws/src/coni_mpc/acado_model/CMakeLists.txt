#     rpg_quadrotor_mpc
#     A model predictive control implementation for quadrotors.
#     Copyright (C) 2017-2018 Philipp Foehn, 
#     Robotics and Perception Group, University of Zurich
#  
#     Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
#     https://github.com/uzh-rpg/rpg_quadrotor_control
# 
#     This program is free software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
# 
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
# 
#     You should have received a copy of the GNU General Public License
#     along with this program.  If not, see <http://www.gnu.org/licenses/>.


# Minimum required version of cmake 
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Project name and programming languages used
PROJECT( quadrotor_model )

# CMake module(s) path
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )
# activate c++ 11
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

#
# Prerequisites
#
FIND_PACKAGE( ACADO REQUIRED )

if(DEFINED ENV{ACADO_ROOT} AND NOT "$ENV{ACADO_ROOT}" STREQUAL "")
  set(LOCAL_ACADO_ROOT "$ENV{ACADO_ROOT}")
else()
  set(LOCAL_ACADO_ROOT "/home/jjm/桌面/ACADOtoolkit")
endif()

#
# Include directories
#
#INCLUDE_DIRECTORIES( . ${ACADO_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES(
  ${LOCAL_ACADO_ROOT}
  ${LOCAL_ACADO_ROOT}/acado
  ${LOCAL_ACADO_ROOT}/acado/include)

LINK_DIRECTORIES(
  ${LOCAL_ACADO_ROOT}/build/lib
  ${LOCAL_ACADO_ROOT}/lib)
#
# Build an executable
#
ADD_EXECUTABLE(        quadrotor_model_codegen quadrotor_model_thrustrates.cpp )
TARGET_LINK_LIBRARIES( quadrotor_model_codegen
  ${ACADO_SHARED_LIBRARIES}
  acado_toolkit_s
  acado_qpoases
  acado_casadi
  acado_csparse )
SET_TARGET_PROPERTIES( quadrotor_model_codegen PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )
