cmake_minimum_required(VERSION 3.16)
project(pibt2 CXX)

# Collect all source files from src
file(GLOB SRCS "./src/*.cpp")

# Create the mapf executable with all sources
add_executable(mapf mapf.cpp ${SRCS})
target_compile_features(mapf PUBLIC cxx_std_17)
target_include_directories(mapf PRIVATE ./include)
target_compile_options(mapf PRIVATE -O3 -Wall -mtune=native -march=native)
add_definitions(-D_MAPDIR_="${CMAKE_CURRENT_LIST_DIR}/map/")

# format
add_custom_target(clang-format
  COMMAND clang-format -i
  ./include/*.hpp
  ./src/*.cpp
  ./mapf.cpp)
