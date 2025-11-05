cmake_minimum_required(VERSION 3.14)

project(module_two_lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SRCS src/ValueHandlerImpl.cpp)

add_library(module_two_lib STATIC ${SRCS})
target_include_directories(
  module_two_lib
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
         $<INSTALL_INTERFACE:include/ros_node_blueprint/module_two>)
target_compile_features(module_two_lib PRIVATE cxx_std_17)

include(GNUInstallDirs)
install(
  TARGETS module_two_lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ros_node_blueprint/module_two)

install(
  DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ros_node_blueprint/module_two
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
