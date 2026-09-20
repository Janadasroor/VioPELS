cmake_minimum_required(VERSION 3.21)
project(power_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(POWER_ENGINE_BUILD_TESTS "Build tests" ON)
option(POWER_ENGINE_BUILD_EXAMPLES "Build examples" ON)
# Sanitizers for CI/local verification: -DPOWER_ENGINE_SANITIZE=address,undefined
set(POWER_ENGINE_SANITIZE "" CACHE STRING "Sanitizers to enable (address,undefined,thread)")

# Warnings
if(MSVC)
  add_compile_options(/W4 /WX)
else()
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Eigen
find_package(Eigen3 3.4 QUIET)
if(NOT Eigen3_FOUND)
  include(FetchContent)
  FetchContent_Declare(eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0)
  FetchContent_MakeAvailable(eigen)
endif()

set(POWER_ENGINE_SOURCES
  src/core/circuit.cpp
  src/solver/solver.cpp
  src/api/engine.cpp
  src/netlist/parser.cpp
  src/control/control.cpp
  src/thermal/thermal.cpp
  src/ac/ac.cpp
)

add_library(power_engine STATIC ${POWER_ENGINE_SOURCES})
add_library(power_engine_shared SHARED ${POWER_ENGINE_SOURCES})
set_target_properties(power_engine_shared PROPERTIES OUTPUT_NAME power_engine)

foreach(tgt power_engine power_engine_shared)
  target_include_directories(${tgt} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  if(Eigen3_FOUND)
    target_link_libraries(${tgt} PUBLIC Eigen3::Eigen)
  else()
    target_link_libraries(${tgt} PUBLIC eigen)
  endif()
  if(POWER_ENGINE_SANITIZE)
    target_compile_options(${tgt} PUBLIC -fno-omit-frame-pointer -fsanitize=${POWER_ENGINE_SANITIZE})
    target_link_options(${tgt} PUBLIC -fsanitize=${POWER_ENGINE_SANITIZE})
  endif()
endforeach()

include(GNUInstallDirs)
install(DIRECTORY include/power_engine DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS power_engine power_engine_shared
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(POWER_ENGINE_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(POWER_ENGINE_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
