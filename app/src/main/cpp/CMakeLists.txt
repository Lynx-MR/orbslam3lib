cmake_minimum_required(VERSION 3.22.1)

project("androidorbslam3")

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Ofast -mstrict-align -mcpu=cortex-a77+crc+simd -ffast-math -ftree-vectorize -funroll-loops -flto=auto")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -mstrict-align -mcpu=cortex-a77+crc+simd -ffast-math -ftree-vectorize -funroll-loops -flto=auto")

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto=auto")

if(NOT DEFINED ANDROID_ABI)
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
endif()
option(USE_GRAPHICS "Enable using graphincs" OFF)
if(USE_GRAPHICS)
    message(STATUS "ADD USE_GRAPHICS")
    add_definitions(-DUSE_GRAPHICS)
endif()

# Require C++17 support
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # Disable compiler-specific extensions (optional, but recommended for portability)

# Add definitions for compatibility (adapted from original; adjust if needed)
add_definitions(-DCOMPILEDWITHC11)  # Retained for potential macros
add_definitions(-DCOMPILEDWITHC17)
message(STATUS "Requiring C++17 standard.")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(OPENCV_SDK_URL "https://github.com/opencv/opencv/releases/download/4.2.0/opencv-4.2.0-android-sdk.zip")
set(OPENCV_SDK_DIR "${PROJECT_SOURCE_DIR}/Thirdparty/OpenCV-android-sdk")

if(NOT EXISTS "${OPENCV_SDK_DIR}")
    message(STATUS "Downloading OpenCV Android SDK...")
    set(DOWNLOAD_PATH "${CMAKE_BINARY_DIR}/opencv_sdk.zip")
    file(DOWNLOAD ${OPENCV_SDK_URL} ${DOWNLOAD_PATH} SHOW_PROGRESS STATUS download_status)
    list(GET download_status 0 status_code)
    if(NOT status_code EQUAL 0)
        message(FATAL_ERROR "Failed to download OpenCV SDK: ${download_status}")
    endif()
    file(ARCHIVE_EXTRACT INPUT ${DOWNLOAD_PATH} DESTINATION ${PROJECT_SOURCE_DIR}/Thirdparty)
    file(REMOVE ${DOWNLOAD_PATH})  # Clean up ZIP after extract
endif()

set(OpenCV_DIR "${PROJECT_SOURCE_DIR}/Thirdparty/OpenCV-android-sdk/sdk/native/jni/abi-arm64-v8a")

find_package(OpenCV 4.2)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 4.2 not found.")
endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

set(EIGEN3_VERSION_OK 3.4.0)
set(EIGEN3_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/Thirdparty/eigen-3.4.0")


include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/CameraModels
        ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/include  # DBoW2 includes
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/include/DBoW2  # DBoW2 includes
)

# Glob DBoW2 sources for static embedding (adjust if structure differs)
file(GLOB DBoW2_SOURCES
        "${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/src/*.cpp"
)

add_library(${PROJECT_NAME} SHARED
        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Atlas.cc
        src/Map.cc
        src/Optimizer.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/ImuTypes.cc
        src/G2oTypes.cc
        src/CameraModels/Pinhole.cpp
        src/CameraModels/KannalaBrandt8.cpp
        src/OptimizableTypes.cpp
        src/MLPnPsolver.cpp
        src/GeometricTools.cc
        src/TwoViewReconstruction.cc
        src/Config.cc
        src/Settings.cc
        src/LynxHardwareAcceleration/LynxHardwareAccelerator.cpp
        src/LynxHardwareAcceleration/FrameAHB.cc
        ${CMAKE_CURRENT_BINARY_DIR}/orbslam3_stub.c
        ${DBoW2_SOURCES}  # Embed DBoW2 statically
)

set(G2O_BUILD_APPS OFF)
set(G2O_BUILD_EXAMPLES OFF)
set(G2O_USE_OPENGL OFF)
set(G2O_USE_CSPARSE OFF)
set(G2O_USE_CHOLMOD OFF)
set(BUILD_SHARED_LIBS OFF)
add_subdirectory(Thirdparty/g2o)
set(BUILD_SHARED_LIBS ON)


include_directories(${g2o_BINARY_DIR})

include_directories(${PROJECT_SOURCE_DIR}/Thirdparty/g2o)

# Hexagon SDK
# Set HEXAGON_SDK_ROOT and HEXAGON_CMAKE_ROOT
set(DSP_VERSION "v66")
set(HEXAGON_SDK_ROOT $ENV{HEXAGON_SDK_ROOT})
message("HEXAGON_SDK_ROOT ${HEXAGON_SDK_ROOT}")
set(HEXAGON_CMAKE_ROOT "${HEXAGON_SDK_ROOT}/build/cmake")
set(PREBUILT_LIB_DIR "android_aarch64")

# Add necessary includes for the project
set(common_incs
        ${CMAKE_CURRENT_BINARY_DIR}/
        ${HEXAGON_SDK_ROOT}/incs/
        ${HEXAGON_SDK_ROOT}/incs/stddef/
        ${HEXAGON_SDK_ROOT}/ipc/fastrpc/rpcmem/inc/
        ${HEXAGON_SDK_ROOT}/utils/examples
)

if(HEXAGON_SDK_ROOT)
    include(${HEXAGON_SDK_ROOT}/build/cmake/hexagon_fun.cmake)
else()
    include(${HEXAGON_CMAKE_ROOT}/hexagon_fun.cmake)
endif()

# Add include directories to the build
include_directories(${common_incs})

# Build interface files using user IDL
build_idl(inc/orbslam3.idl ${PROJECT_NAME})

# Prepare and link with remote library
target_link_directories(${PROJECT_NAME} PUBLIC "${HEXAGON_SDK_ROOT}/ipc/fastrpc/remote/ship/${PREBUILT_LIB_DIR}")
target_link_libraries(${PROJECT_NAME} cdsprpc)

# Prepare and link with rpcmem library
link_custom_library(${PROJECT_NAME} rpcmem)


target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        g2o::types_sba
        g2o::types_sim3
)

target_link_libraries(${CMAKE_PROJECT_NAME}
        android
        log
        glesv3
        egl
)