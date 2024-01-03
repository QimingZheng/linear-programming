cmake_minimum_required(VERSION 3.24)

project(
  lp
  VERSION 1.0
  LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 11)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)
add_library(GTest::GTest INTERFACE IMPORTED)
target_link_libraries(GTest::GTest INTERFACE gtest_main gmock_main)

include_directories(lp_header lp.h)
add_library(lp_library SHARED lp.cc)

enable_testing()

add_executable(
  lp_test
  lp_test.cc
)
target_include_directories(lp_test PRIVATE lp_header)
target_link_libraries(
  lp_test
  GTest::GTest
  lp_library
)

add_test(
  NAME test
  COMMAND lp_test
)