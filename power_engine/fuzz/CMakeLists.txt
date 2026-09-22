# libFuzzer target for the netlist parser. Opt-in: needs Clang.
#   cmake -B build-fuzz -DCMAKE_CXX_COMPILER=clang++ \\
#     -DPOWER_ENGINE_BUILD_FUZZ=ON -DPOWER_ENGINE_SANITIZE=address \\
#     -DPOWER_ENGINE_BUILD_TESTS=OFF -DPOWER_ENGINE_BUILD_EXAMPLES=OFF
#   cmake --build build-fuzz --target fuzz_netlist
#   ./build-fuzz/power_engine/fuzz/fuzz_netlist power_engine/fuzz/corpus -max_len=1024
#   ./build-fuzz/power_engine/fuzz/fuzz_netlist power_engine/fuzz/corpus -max_len=1024 -jobs=$(nproc)  # extended
if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  message(FATAL_ERROR "POWER_ENGINE_BUILD_FUZZ needs Clang (libFuzzer)")
endif()
add_executable(fuzz_netlist fuzz_netlist.cpp)
target_link_libraries(fuzz_netlist PRIVATE power_engine)
target_compile_options(fuzz_netlist PRIVATE -fsanitize=fuzzer,address -g -O1)
target_link_options(fuzz_netlist PRIVATE -fsanitize=fuzzer,address)
