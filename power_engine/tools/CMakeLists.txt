# pe command framework + subcommands (testable core, no main()).
# pe_cli builds always (tests link it); the pe binary follows the
# examples gate (user-facing extra like the demos).
add_library(pe_cli STATIC
  pe/pe.cpp
  pe/cmd_run.cpp
)
target_include_directories(pe_cli PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(pe_cli PUBLIC power_engine power_engine_warnings)

if(POWER_ENGINE_BUILD_EXAMPLES)
  add_executable(pe pe/pe_main.cpp)
  target_link_libraries(pe PRIVATE pe_cli power_engine_warnings)
  target_compile_definitions(pe PRIVATE PE_VERSION="${VioPELS_VERSION}")
  install(TARGETS pe RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
