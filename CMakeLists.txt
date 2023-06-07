# project name and language
cmake_minimum_required(VERSION 3.0.0)
project(soem_interface LANGUAGES CXX)

find_package(spdlog REQUIRED)

add_subdirectory(SOEM)

file(GLOB ECM_SOURCES ${CMAKE_CURRENT_LIST_DIR}/src/*)

file(GLOB ECM_HEADERS ${CMAKE_CURRENT_LIST_DIR}/include/soem_interface/*)

include_directories(
  include
)

add_library(soem_interface
  src/soem_interface/common/ThreadSleep.cpp
  src/soem_interface/common/Macros.cpp
  src/soem_interface/EthercatSlaveBase.cpp
  src/soem_interface/EthercatBusManagerBase.cpp
  src/soem_interface/EthercatBusBase.cpp
)


target_sources(soem_interface
  PRIVATE
    ${ECM_SOURCES}
  PUBLIC
    ${ECM_HEADERS}
)

target_link_libraries(soem_interface soem)

target_include_directories(soem_interface
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include/
    /SOEM/
  )

add_subdirectory(soem_interface_drivers)