cmake_minimum_required(VERSION 3.16)
project(pibt2 CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define include directory
set(PIBT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Collect all source files from src (excluding Python binding files)
file(GLOB SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
list(REMOVE_ITEM SRCS 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/python_bindings.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/py_pibt.cpp"
)

# Create the mapf executable with all sources
add_executable(mapf mapf.cpp ${SRCS})
target_compile_features(mapf PUBLIC cxx_std_17)
target_include_directories(mapf PRIVATE ${PIBT_INCLUDE_DIR})
target_compile_options(mapf PRIVATE -O3 -Wall -mtune=native -march=native)
target_compile_definitions(mapf PRIVATE _MAPDIR_="${CMAKE_CURRENT_LIST_DIR}/map/")

# format
add_custom_target(clang-format
  COMMAND clang-format -i
  ./include/*.hpp
  ./src/*.cpp
  ./mapf.cpp)
