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
include_directories(ilp_header ilp.h)
include_directories(base_header base.h)
add_library(lp_library SHARED lp.cc)
add_library(ilp_library SHARED ilp.cc)
target_link_libraries(ilp_library lp_library)

add_executable(solver solver.cc)
target_include_directories(solver PRIVATE ilp_header lp_header base_header)
target_link_libraries(solver lp_library ilp_library)

enable_testing()

add_executable(
  lp_test
  lp_test.cc
)
target_include_directories(lp_test PRIVATE lp_header base_header)
target_link_libraries(
  lp_test
  GTest::GTest
  lp_library
)

add_executable(
  ilp_test
  ilp_test.cc
)
target_include_directories(ilp_test PRIVATE ilp_header lp_header base_header)
target_link_libraries(
  ilp_test
  GTest::GTest
  ilp_library
)

add_test(
  NAME ilp_test
  COMMAND ilp_test
)

add_test(
  NAME lp_test
  COMMAND lp_test
)

add_executable(
  base_test
  base_test.cc
)
target_include_directories(base_test PRIVATE base_header)
target_link_libraries(
  base_test
  GTest::GTest
)

add_test(
  NAME base_test
  COMMAND base_test
)