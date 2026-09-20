cmake_minimum_required(VERSION 3.21)
project(VioPELS LANGUAGES CXX VERSION 0.6.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Unoptimized Eigen is ~17x slower (measured: buck 6ms sim 0.65s -> 0.037s),
# so default to Release when the user says nothing.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    Debug Release RelWithDebInfo MinSizeRel)
endif()
message(STATUS "VioPELS build type: ${CMAKE_BUILD_TYPE}")
enable_testing()
add_subdirectory(power_engine)
