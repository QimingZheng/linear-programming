cmake_minimum_required(VERSION 3.24)

project(
  lp
  VERSION 1.0
  LANGUAGES CXX)

find_package (Eigen3 REQUIRED NO_MODULE)
 
set (CMAKE_CXX_STANDARD 17)
string(REPLACE "-DNDEBUG" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

find_package(OpenMP)

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

file(GLOB HEADERS *.h)
file(GLOB TEST_SRCS *_test.cc)
file(GLOB SRCS *.cc)
list(FILTER SRCS EXCLUDE REGEX ".*test\.cc$")
list(FILTER SRCS EXCLUDE REGEX "solver\.cc")
# TODO: remove this exclusion when DW solver is ready.
list(FILTER SRCS EXCLUDE REGEX "lp_dantzig_wolfe_decomposition\.cc")
# TODO: fix the bug of primal-dual path following solver.
list(FILTER TEST_SRCS EXCLUDE REGEX "lp_primal_dual_path_following_test\.cc")

include_directories(headers ${CMAKE_CURRENT_SOURCE_DIR})
add_library(lp_library SHARED ${SRCS})
target_link_libraries(lp_library Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
    target_link_libraries(lp_library OpenMP::OpenMP_CXX)
endif()

add_executable(solver solver.cc)
target_include_directories(solver PRIVATE headers)
target_link_libraries(solver lp_library)

enable_testing()

foreach(test_src ${TEST_SRCS})
  string(FIND ${test_src} "/" start REVERSE)
  string(FIND ${test_src} ".cc" end)
  MATH(EXPR begin "${start}+1")
  MATH(EXPR length "${end}-${start}-1")
  string(SUBSTRING ${test_src} ${begin} ${length} test_name)
  add_executable(
    ${test_name}
    ${test_src}
  )
  target_include_directories(${test_name} PRIVATE headers)
  target_link_libraries(
    ${test_name}
    GTest::GTest
    lp_library
  )
  add_test(
    NAME ${test_name}
    COMMAND ${test_name}
  )
endforeach()

set(TestCases 12 13 14 15 16 17)
foreach(Case ${TestCases})
  add_test(NAME TestSimplex${Case} COMMAND ./solver tests/test${Case}.txt simplex)
  file(READ tests/sol${Case}.txt Solution)
  set_tests_properties(TestSimplex${Case} PROPERTIES PASS_REGULAR_EXPRESSION ${Solution})
endforeach()

# TODO: add test cases 15 16 17 after phase 1 of simplex-tableau solver is implemented.
set(TestCases 12 13 14 15 16 17)
foreach(Case ${TestCases})
  add_test(NAME TestSimplexTableau${Case} COMMAND ./solver tests/test${Case}.txt simplex_tableau)
  file(READ tests/sol${Case}.txt Solution)
  if (EXISTS tests/sol${Case}_simplex_tableau.txt)
    file(READ tests/sol${Case}_simplex_tableau.txt Solution)
  endif()
  set_tests_properties(TestSimplexTableau${Case} PROPERTIES PASS_REGULAR_EXPRESSION ${Solution})
endforeach()

set(TestCases 12 13 14 15 16 17)
foreach(Case ${TestCases})
  add_test(NAME TestColumnGeneration${Case} COMMAND ./solver tests/test${Case}.txt column_generation)
  file(READ tests/sol${Case}.txt Solution)
  set_tests_properties(TestColumnGeneration${Case} PROPERTIES PASS_REGULAR_EXPRESSION ${Solution})
endforeach()