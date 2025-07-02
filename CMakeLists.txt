cmake_minimum_required(VERSION 3.16)
project(pibt2 CXX)

add_subdirectory(./pibt2)
add_subdirectory(./third_party/googletest)

add_executable(mapf mapf.cpp)
target_compile_features(mapf PUBLIC cxx_std_17)
target_link_libraries(mapf lib-mapf)

# format
add_custom_target(clang-format
  COMMAND clang-format -i
  ../pibt2/include/*.hpp
  ../pibt2/src/*.cpp
  ../tests/*.cpp
  ../mapf.cpp)

# test
set(TEST_MAIN_FUNC ./third_party/googletest/googletest/src/gtest_main.cc)
set(TEST_ALL_SRC ${TEST_MAIN_FUNC})
macro(add_test name target)
  add_executable(${name} ${target} ${TEST_MAIN_FUNC})
  target_link_libraries(${name} lib-mapf gtest)
  list(APPEND TEST_ALL_SRC ${target})
endmacro(add_test)

# basic
add_test(test_plan ./tests/test_plan.cpp)
add_test(test_paths ./tests/test_paths.cpp)
add_test(test_solver ./tests/test_solver.cpp)
add_test(test_problem ./tests/test_problem.cpp)
add_test(test_pibt ./tests/test_pibt.cpp)

add_executable(test ${TEST_ALL_SRC})
target_link_libraries(test lib-mapf gtest)
