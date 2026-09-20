cmake_minimum_required(VERSION 3.16)
project(OPenumbra LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(openumbra_octree_demo src/main.cpp)
target_include_directories(openumbra_octree_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

enable_testing()
add_test(NAME openumbra_octree_demo COMMAND openumbra_octree_demo)
