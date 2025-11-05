cmake_minimum_required(VERSION 3.14)

add_subdirectory(ModuleOne)
add_subdirectory(ModuleTwo)
add_subdirectory(ModuleThree)

add_library(blueprint_core INTERFACE)
target_link_libraries(blueprint_core INTERFACE module_one_lib module_two_lib
                                               module_three_lib)
