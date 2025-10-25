# Enable testing
enable_testing()
find_package(GTest REQUIRED)

# Test executable
add_executable(plant_user_test
    plant_user_test.cc
    $<TARGET_OBJECTS:plant_user_obj>
)

target_include_directories(plant_user_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../..   # to reach plant_user_api.h
)

target_link_libraries(plant_user_test
    PRIVATE GTest::gtest GTest::gtest_main m
)

gtest_discover_tests(plant_user_test
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DISCOVERY_TIMEOUT 30
)