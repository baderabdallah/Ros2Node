cmake_minimum_required(VERSION 3.14)

project(module_one_lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SRCS src/GenericFilter.cpp)

add_library(module_one_lib STATIC ${SRCS})
target_include_directories(
  module_one_lib
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
         $<INSTALL_INTERFACE:include/ros_node_blueprint/module_one>)
target_compile_features(module_one_lib PRIVATE cxx_std_17)

include(GNUInstallDirs)
install(
  TARGETS module_one_lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ros_node_blueprint/module_one)

install(
  DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ros_node_blueprint/module_one
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

# Tests
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
