# heapsx-box2d: Box2D Physics Engine Bindings for HashLink/Heaps.io

cmake_minimum_required(VERSION 3.22.1)

project(heapsx-box2d)

# Use Box2D library (git submodule)
set(BOX2D_DIR "${CMAKE_CURRENT_SOURCE_DIR}/box2d")

if(NOT EXISTS "${BOX2D_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "Box2D library not found at ${BOX2D_DIR}. Run: git submodule update --init")
endif()

message(STATUS "heapsx-box2d: Using Box2D from ${BOX2D_DIR}")

# Build Box2D library first
set(BOX2D_BUILD_DOCS OFF CACHE BOOL "Don't build Box2D documentation")
set(BOX2D_BUILD_SAMPLES OFF CACHE BOOL "Don't build Box2D samples")
set(BOX2D_BUILD_TESTBED OFF CACHE BOOL "Don't build Box2D testbed")
set(BOX2D_BUILD_UNIT_TESTS OFF CACHE BOOL "Don't build Box2D unit tests")
set(BOX2D_USER_SETTINGS OFF CACHE BOOL "Don't use Box2D user settings")

# Add Box2D as subdirectory
add_subdirectory(${BOX2D_DIR} ${CMAKE_BINARY_DIR}/box2d-build EXCLUDE_FROM_ALL)

# CRITICAL: Build with -fPIC for shared library linking
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Dynamic vs Static build for heapsx-box2d wrapper
if(BUILD_HEAPSX_DYNAMIC)
    # Dynamic: Build as .hdll (shared library for HashLink VM)
    add_library(heapsx-box2d SHARED
        box2d_bindings.cpp
    )

    # Android vs Desktop naming
    if(ANDROID)
        set_target_properties(heapsx-box2d PROPERTIES
            OUTPUT_NAME "box2d.hdll"
            PREFIX "lib"
            SUFFIX ".so"
            CXX_STANDARD 17
            CXX_STANDARD_REQUIRED ON
        )
        message(STATUS "heapsx-box2d: Building as libbox2d.hdll.so for Android")
    else()
        set_target_properties(heapsx-box2d PROPERTIES
            OUTPUT_NAME "box2d"
            PREFIX ""
            SUFFIX ".hdll"
            CXX_STANDARD 17
            CXX_STANDARD_REQUIRED ON
        )
        message(STATUS "heapsx-box2d: Building as box2d.hdll for Desktop")
    endif()
else()
    # Static: Build as .a (linked into main binary)
    add_library(heapsx-box2d STATIC
        box2d_bindings.cpp
    )
    set_target_properties(heapsx-box2d PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
    )
    message(STATUS "heapsx-box2d: Building as static .a")
endif()

# HashLink includes
if(DEFINED HASHLINK_DIR)
    target_include_directories(heapsx-box2d PRIVATE
        ${HASHLINK_DIR}/src
        ${HASHLINK_DIR}/include
    )
else()
    message(WARNING "heapsx-box2d: HASHLINK_DIR not defined")
endif()

# Box2D includes (provided by the box2d target)
target_include_directories(heapsx-box2d PUBLIC
    ${BOX2D_DIR}/include
)

# Link against Box2D static library
target_link_libraries(heapsx-box2d PRIVATE box2d)

# Android-specific libraries
if(ANDROID)
    target_link_libraries(heapsx-box2d PRIVATE log)
endif()

message(STATUS "heapsx-box2d: Module configured successfully")
