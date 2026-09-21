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
if (UNIX AND NOT APPLE)
 target_compile_options(MaskedOcclusionCulling PRIVATE -mxsave)
    file(READ "${maskedocclusionculling_SOURCE_DIR}/CompilerSpecific.inl" MOC_COMPILER_SPECIFIC)
    string(REPLACE "FORCE_INLINE void __cpuidex(int* cpuinfo, int function, int subfunction)\n\t{\n\t\t__cpuid_count(function, subfunction, cpuinfo[0], cpuinfo[1], cpuinfo[2], cpuinfo[3]);\n\t}\n\n" "" MOC_COMPILER_SPECIFIC "${MOC_COMPILER_SPECIFIC}")
    string(REPLACE "FORCE_INLINE unsigned long long _xgetbv(unsigned int index)\n\t{\n\t\tunsigned int eax, edx;\n\t\t__asm__ __volatile__(\n\t\t\t\"xgetbv;\"\n\t\t\t: \"=a\" (eax), \"=d\"(edx)\n\t\t\t: \"c\" (index)\n\t\t);\n\t\treturn ((unsigned long long)edx << 32) | eax;\n\t}\n\n" "" MOC_COMPILER_SPECIFIC "${MOC_COMPILER_SPECIFIC}")
    file(WRITE "${maskedocclusionculling_SOURCE_DIR}/CompilerSpecific.inl" "${MOC_COMPILER_SPECIFIC}")
endif()

add_library(openu STATIC src/OctreeRasterizer.cpp)
target_include_directories(openu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(openu PUBLIC MaskedOcclusionCulling)

add_executable(openumbra_octree_demo src/main.cpp)
target_link_libraries(openumbra_octree_demo PRIVATE openu)

add_executable(openu_octree_rasterizer_test tests/OctreeRasterizerTest.cpp)
target_link_libraries(openu_octree_rasterizer_test PRIVATE openu)

enable_testing()
add_test(NAME openumbra_octree_demo COMMAND openumbra_octree_demo)
add_test(NAME openu_octree_rasterizer COMMAND openu_octree_rasterizer_test)
