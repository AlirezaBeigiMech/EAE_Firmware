find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(ctrl_set_tests
  ctrl_set_test.cc
  $<TARGET_OBJECTS:ctrl_set_obj>    # links the ctrl_set.c object compiled with -DUNIT_TEST
)

# So the test can see your headers next to sources (ctrl_set_api.h, etc.)
target_include_directories(ctrl_set_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../..   # back to project root
)

target_link_libraries(ctrl_set_tests PRIVATE GTest::gtest GTest::gtest_main m)

# Register with CTest (auto-discovers TEST() cases)
gtest_discover_tests(ctrl_set_tests)