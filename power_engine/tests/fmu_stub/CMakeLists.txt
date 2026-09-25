# Test stub FMU (FMI 2.0 CS bang-bang controller): stages a realistic
# extracted-FMU layout (modelDescription.xml + binaries/<plat>/<id>.<ext>)
# so the import tests exercise the production path, not a shortcut.
# The top project is CXX-only; the stub is deliberate C (FMUs are C ABI).
include(CheckLanguage)
check_language(C)
if(NOT CMAKE_C_COMPILER)
  message(FATAL_ERROR "fmu_stub needs a C compiler")
endif()
enable_language(C)
add_library(fmu_stub MODULE stub.c)
target_include_directories(fmu_stub PRIVATE ${CMAKE_SOURCE_DIR}/power_engine/fmi/include)
set_target_properties(fmu_stub PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/stage
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/stage
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/stage)

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
  set(_fmu_plat win64)
  set(_fmu_ext .dll)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  set(_fmu_plat darwin64)
  set(_fmu_ext .dylib)
else()
  set(_fmu_plat linux64)
  set(_fmu_ext .so)
endif()

set(_fmu_unpack ${CMAKE_CURRENT_BINARY_DIR}/stage/fmu_unpack)
add_custom_command(TARGET fmu_stub POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory ${_fmu_unpack}/binaries/${_fmu_plat}
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:fmu_stub>
          ${_fmu_unpack}/binaries/${_fmu_plat}/fmu_stub${_fmu_ext}
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/modelDescription.xml
          ${_fmu_unpack}/modelDescription.xml)
