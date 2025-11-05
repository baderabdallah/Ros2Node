cmake_minimum_required(VERSION 3.14)

include(CTest)
if(NOT BUILD_TESTING)
  return()
endif()

find_package(GTest QUIET)
if(NOT TARGET GTest::gtest_main)
  include(FetchContent)
  if(NOT gtest_POPULATED)
    FetchContent_Declare(
      gtest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG v1.13.0)
    FetchContent_MakeAvailable(gtest)
  endif()
endif()

add_executable(module_three_tests tests_module_three.cpp)
target_link_libraries(module_three_tests PRIVATE module_three_lib
                                                 GTest::gtest_main)
target_include_directories(module_three_tests
                           PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../include)
target_compile_features(module_three_tests PRIVATE cxx_std_17)

include(GoogleTest)
gtest_discover_tests(module_three_tests PROPERTIES LABELS module_three)
