cmake_minimum_required(VERSION 3.16)
project(OPenumbra LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

# MaskedOcclusionCulling is an archived Intel project. Keep the exact revision so
# builds do not silently change when the upstream repository changes.
set(USE_AVX512 OFF CACHE BOOL "Disable experimental AVX-512 support in MOC" FORCE)
FetchContent_Declare(
    MaskedOcclusionCulling
    GIT_REPOSITORY https://github.com/GameTechDev/MaskedOcclusionCulling.git
    GIT_TAG 1fd7974456cffa481a1a534328a1d02523d19ce8
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(MaskedOcclusionCulling)

add_library(openu STATIC src/OctreeRasterizer.cpp)
target_include_directories(openu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(openu PUBLIC MaskedOcclusionCulling)

add_executable(openumbra_octree_demo src/main.cpp)
target_link_libraries(openumbra_octree_demo PRIVATE openu)

enable_testing()
add_test(NAME openumbra_octree_demo COMMAND openumbra_octree_demo)
