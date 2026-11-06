add_executable(pe_bench bench.cpp)
target_link_libraries(pe_bench PRIVATE power_engine power_engine_warnings)

add_executable(pe_scaling scaling.cpp)
target_link_libraries(pe_scaling PRIVATE power_engine power_engine_warnings)
