# Tests for ros_node_blueprint

find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)

ament_add_gtest(test_blueprint_node BluePrintNodeTestFixture.cpp
                BluePrintNodeTest.cpp)

if(TARGET test_blueprint_node)
  ament_target_dependencies(test_blueprint_node rclcpp ${ROS_DEPS})
  target_link_libraries(test_blueprint_node ${LIBRARY_NAME})
  target_include_directories(test_blueprint_node
                             PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
endif()
