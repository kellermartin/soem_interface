# Copyright (2019-2020) Robotics Systems Lab - ETH Zurich:
# Markus Staeuble, Jonas Junger, Johannes Pankert, Philipp Leemann,
# Tom Lankhorst, Samuel Bachmann, Gabriel Hottiger, Lennert Nachtigall,
# Mario Mauerer, Remo Diethelm
#
# This file is part of the soem_interface.
#
# The soem_interface is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# The seom_interface is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with the soem_interface.  If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.0.0)
project(soem_interface_drivers)

add_definitions(-std=c++14 -Wall -Werror)


file(GLOB ECM_HEADERS ${CMAKE_CURRENT_LIST_DIR}/include/*)

include_directories(
  include
  ../SOEM/soem
  ../SOEM/osal
  ../SOEM/osal/linux
  ../SOEM/oshw/linux
  soem_interface
)

add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/ExampleSlave.cpp
  src/${PROJECT_NAME}/EL1004.cpp
  src/${PROJECT_NAME}/EL1008.cpp
  src/${PROJECT_NAME}/EL2004.cpp
  src/${PROJECT_NAME}/EL2008.cpp
  src/${PROJECT_NAME}/Lenze_I550.cpp 
)

target_link_libraries(
  ${PROJECT_NAME}
  soem_interface
  soem
)

add_executable(
  example
  src/example_slave.cpp
)

target_link_libraries(example
  soem_interface_drivers
  soem_interface
  soem
  spdlog::spdlog
)

target_sources(soem_interface_drivers
  PUBLIC
    ${ECM_HEADERS}
)