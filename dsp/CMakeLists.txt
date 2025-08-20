# Sets the minimum version of CMake required to build your native library.
# This ensures that a certain set of CMake features is available to
# your build.

cmake_minimum_required(VERSION 3.14.3)

project("orbslam3" C CXX ASM)

enable_language(ASM)

if(HEXAGON_SDK_ROOT)
    include(${HEXAGON_SDK_ROOT}/build/cmake/hexagon_fun.cmake)
else()
    include(${HEXAGON_CMAKE_ROOT}/hexagon_fun.cmake)
endif()

set(common_incs
        ${CMAKE_CURRENT_BINARY_DIR}/
        ${HEXAGON_SDK_ROOT}/incs/
        # ${HEXAGON_SDK_ROOT}/incs/stddef/
        # ${HEXAGON_SDK_ROOT}/utils/examples
        ${HEXAGON_SDK_ROOT}/rtos/qurt/
        )

include_directories(${common_incs})
include_directories(src/)

set(srcs
        ${CMAKE_CURRENT_BINARY_DIR}/orbslam3_skel.c
        src/orbslam_dsp_fast.cpp
        src/orbslam_dsp_matcher.cpp 
        src/orbslam_dsp_pyramid.cpp
        src/orbslam_dsp_rb.cpp
        src/orbslam_dsp_sort.cpp
        src/orbslam_dsp.cpp
    )

add_library(orbslam3_skel SHARED ${srcs})

add_library(orbslam3hwa_skel STATIC IMPORTED)
set_target_properties(orbslam3hwa_skel PROPERTIES
    IMPORTED_LOCATION "../lib/liborbslam3hwa_skel.a"
)

target_link_libraries(orbslam3_skel PRIVATE orbslam3hwa_skel)

build_idl(../app/src/main/cpp/inc/orbslam3.idl orbslam3_skel)


