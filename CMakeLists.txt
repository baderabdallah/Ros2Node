cmake_minimum_required(VERSION 3.5)
project(ros_node_blueprint)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(std_msgs REQUIRED)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME} "msg/MsgOne.msg")

# Export the runtime dependency for downstream packages
ament_export_dependencies(rosidl_default_runtime)

# define names for node library and node executable
set(EXEC_NAME "blueprint_node")
set(LIBRARY_NAME ${EXEC_NAME}_lib)

set(ROS_DEPS rclcpp rclcpp_action std_msgs rosidl_default_runtime)

# Build local core (defines blueprint_core target)
add_subdirectory(src/core)

# export the node class (with dependent classes) as a shared library
add_library(${LIBRARY_NAME} SHARED src/BluePrintNode.cpp
                                   src/NodeHelperFunctions.cpp)

set_target_properties(
  ${LIBRARY_NAME}
  PROPERTIES POSITION_INDEPENDENT_CODE ON
             BUILD_RPATH_USE_ORIGIN ON
             INSTALL_RPATH_USE_LINK_PATH ON
             INSTALL_RPATH "$ORIGIN")

ament_target_dependencies(${LIBRARY_NAME} ${ROS_DEPS})
target_link_libraries(${LIBRARY_NAME} blueprint_core)
target_include_directories(
  ${LIBRARY_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
                         $<INSTALL_INTERFACE:include>)

# Ensure the generated message interfaces are available to this target (include
# dirs and deps)
rosidl_get_typesupport_target(rosidl_typesupport_cpp_target ${PROJECT_NAME}
                              "rosidl_typesupport_cpp")
target_link_libraries(${LIBRARY_NAME} ${rosidl_typesupport_cpp_target}
                      ${PROJECT_NAME}__rosidl_generator_cpp)

# create the node executable
add_executable(${EXEC_NAME} src/BluePrintMain.cpp)
ament_target_dependencies(${EXEC_NAME} ${ROS_DEPS})
target_link_libraries(
  ${EXEC_NAME} ${LIBRARY_NAME} ${rosidl_typesupport_cpp_target}
  ${PROJECT_NAME}__rosidl_generator_cpp)

set_target_properties(
  ${EXEC_NAME}
  PROPERTIES BUILD_RPATH_USE_ORIGIN ON
             INSTALL_RPATH_USE_LINK_PATH ON
             INSTALL_RPATH "$ORIGIN/../../lib")

install(
  TARGETS ${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ${EXEC_NAME} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Testing ###
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  add_subdirectory(src/tests)
endif()

ament_package()
