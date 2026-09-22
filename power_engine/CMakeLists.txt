cmake_minimum_required(VERSION 3.21)
project(power_engine VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(POWER_ENGINE_BUILD_TESTS "Build tests" ON)
option(POWER_ENGINE_BUILD_EXAMPLES "Build examples" ON)
# Sanitizers for CI/local verification: -DPOWER_ENGINE_SANITIZE=address,undefined
set(POWER_ENGINE_SANITIZE "" CACHE STRING "Sanitizers to enable (address,undefined,thread)")

# Warnings-as-errors apply to OUR targets only (never to FetchContent
# dependencies: third-party code must not inherit -Werror/-WX, see CI).
# Consume via: target_link_libraries(<tgt> PRIVATE power_engine_warnings).
add_library(power_engine_warnings INTERFACE)
if(MSVC)
  target_compile_options(power_engine_warnings INTERFACE /W4 /WX)
else()
  target_compile_options(power_engine_warnings INTERFACE -Wall -Wextra -Wpedantic -Werror)
endif()

# Eigen: system package preferred; FetchContent fallback is header-only.
find_package(Eigen3 3.4 QUIET)
if(NOT Eigen3_FOUND)
  include(FetchContent)
  FetchContent_Declare(eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE)
  # Eigen 3.4 gates its 900+ tests on the standard BUILD_TESTING option.
  # Force it OFF for the FetchContent evaluation only. NOTE: this must be a
  # CACHE FORCE (not a normal variable): Eigen's cmake_minimum_required
  # predates CMP0077, so its option() call clears normal variables (seen in
  # CI logs). Restored right after.
  if(DEFINED BUILD_TESTING)
    set(_pe_saved_build_testing "${BUILD_TESTING}")
  endif()
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(eigen)
  if(DEFINED _pe_saved_build_testing)
    set(BUILD_TESTING "${_pe_saved_build_testing}" CACHE BOOL "" FORCE)
  else()
    unset(BUILD_TESTING CACHE)
  endif()
  unset(_pe_saved_build_testing)
  # Third-party headers must not trip our -Werror (GCC/Clang honor SYSTEM;
  # MSVC is additionally guarded by pragma push/pop at the include site).
  get_target_property(_eigen_inc eigen INTERFACE_INCLUDE_DIRECTORIES)
  target_include_directories(eigen SYSTEM INTERFACE ${_eigen_inc})
  unset(_eigen_inc)
endif()

set(POWER_ENGINE_SOURCES
  src/core/circuit.cpp
  src/solver/solver.cpp
  src/api/engine.cpp
  src/netlist/parser.cpp
  src/control/control.cpp
  src/thermal/thermal.cpp
  src/ac/ac.cpp
  src/steadystate/steadystate.cpp
  src/measurements/measurements.cpp
  src/waveforms/waveforms.cpp
  src/loopgain/loopgain.cpp
  src/loss_tables/loss_tables.cpp
  src/electrothermal/electrothermal.cpp
  src/multitone/multitone.cpp
  src/statespace/statespace.cpp
  src/machine/machine.cpp
  src/sweep/sweep.cpp
  src/magnetics/magnetics.cpp
)

add_library(power_engine STATIC ${POWER_ENGINE_SOURCES})
add_library(power_engine_shared SHARED ${POWER_ENGINE_SOURCES})
set_target_properties(power_engine_shared PROPERTIES OUTPUT_NAME power_engine)

foreach(tgt power_engine power_engine_shared)
  target_include_directories(${tgt} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  # Warnings are build-tree-only: PRIVATE deps of static archives would
  # otherwise propagate -Werror to consumers via the export set.
  target_link_libraries(${tgt} PRIVATE $<BUILD_INTERFACE:power_engine_warnings>)
  if(Eigen3_FOUND)
    target_link_libraries(${tgt} PUBLIC Eigen3::Eigen)
  else()
    # Build tree uses the FetchContent target; installs resolve Eigen3::Eigen
    # via power_engineConfig.cmake (system Eigen preferred, bundled headers
    # as fallback) so the export set never references a build-tree target.
    target_link_libraries(${tgt} PUBLIC
      $<BUILD_INTERFACE:eigen> $<INSTALL_INTERFACE:Eigen3::Eigen>)
  endif()
  if(POWER_ENGINE_SANITIZE)
    target_compile_options(${tgt} PUBLIC -fno-omit-frame-pointer -fsanitize=${POWER_ENGINE_SANITIZE})
    target_link_options(${tgt} PUBLIC -fsanitize=${POWER_ENGINE_SANITIZE})
  endif()
endforeach()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
install(DIRECTORY include/power_engine DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS power_engine power_engine_shared EXPORT power_engineTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(EXPORT power_engineTargets
  NAMESPACE PowerEngine::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/power_engine)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/power_engineConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/power_engineConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/power_engine)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/power_engineConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/power_engineConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/power_engineConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/power_engine)
if(NOT Eigen3_FOUND)
  # No system Eigen: bundle the FetchContent headers so the installed
  # package is self-contained (the config prefers system Eigen first).
  install(DIRECTORY "${FETCHCONTENT_SOURCE_DIR_EIGEN}/Eigen"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()

if(POWER_ENGINE_BUILD_EXAMPLES)
  add_subdirectory(examples)
  add_subdirectory(bench)
endif()

if(POWER_ENGINE_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
