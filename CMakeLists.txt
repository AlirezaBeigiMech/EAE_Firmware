cmake_minimum_required(VERSION 3.15)
project(PlantUserProject C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

enable_testing()

# Object for plant_user
add_library(plant_user_obj OBJECT plant_user.c)
target_compile_definitions(plant_user_obj PRIVATE UNIT_TEST)
target_include_directories(plant_user_obj PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Object for ctrl_set
add_library(ctrl_set_obj OBJECT ctrl_set.c)
target_compile_definitions(ctrl_set_obj PRIVATE UNIT_TEST)
target_include_directories(ctrl_set_obj PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

enable_testing()
# Add subdirectory for tests
add_subdirectory(unit_test)
