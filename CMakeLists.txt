cmake_minimum_required(VERSION 3.24)

project(
  lp
  VERSION 1.0
  LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 11)
string(REPLACE "-DNDEBUG" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)
add_library(GTest::GTest INTERFACE IMPORTED)
target_link_libraries(GTest::GTest INTERFACE gtest_main gmock_main)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/tests/
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/tests/)

include_directories(headers base.h parser.h ilp.h lp.h)
add_library(lp_library SHARED lp.cc ilp.cc parser.cc base.cc lp_simplex.cc lp_column_generation.cc lp_dual_simplex.cc)

add_executable(solver solver.cc)
target_include_directories(solver PRIVATE headers)
target_link_libraries(solver lp_library)

enable_testing()

add_executable(
  lp_test
  lp_test.cc
)
target_include_directories(lp_test PRIVATE headers)
target_link_libraries(
  lp_test
  GTest::GTest
  lp_library
)

add_test(
  NAME lp_test
  COMMAND lp_test
)

add_executable(
  ilp_test
  ilp_test.cc
)
target_include_directories(ilp_test PRIVATE headers)
target_link_libraries(
  ilp_test
  GTest::GTest
  lp_library
)

add_test(
  NAME ilp_test
  COMMAND ilp_test
)

add_executable(
  base_test
  base_test.cc
)
target_include_directories(base_test PRIVATE headers)
target_link_libraries(
  base_test
  GTest::GTest
  lp_library
)

add_test(
  NAME base_test
  COMMAND base_test
)

add_executable(
  parser_test
  parser_test.cc
)
target_include_directories(parser_test PRIVATE headers)
target_link_libraries(
  parser_test
  GTest::GTest
  lp_library
)

add_test(
  NAME parser_test
  COMMAND parser_test
)

set(TestCases 12 13 14 15 16 17)
foreach(Case ${TestCases})
  add_test(NAME TestSimplex${Case} COMMAND ./solver tests/test${Case}.txt simplex)
  file(READ tests/sol${Case}.txt Solution)
  set_tests_properties(TestSimplex${Case} PROPERTIES PASS_REGULAR_EXPRESSION ${Solution})
endforeach()

set(TestCases 12 13 14 15 16 17)
foreach(Case ${TestCases})
  add_test(NAME TestColumnGeneration${Case} COMMAND ./solver tests/test${Case}.txt column_generation)
  file(READ tests/sol${Case}.txt Solution)
  set_tests_properties(TestColumnGeneration${Case} PROPERTIES PASS_REGULAR_EXPRESSION ${Solution})
endforeach()