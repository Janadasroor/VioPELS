cmake_minimum_required(VERSION 3.21)

set(PE_EXAMPLES rc_demo buck_demo closed_loop_demo netlist_buck_demo ac_demo
  fullbridge_demo vienna_demo dab_demo llc_demo pmsm_demo induction_demo sweep_demo)

add_executable(rc_demo rc_demo.cpp)
add_executable(buck_demo buck_demo.cpp)
add_executable(closed_loop_demo closed_loop_demo.cpp)
add_executable(netlist_buck_demo netlist_buck_demo.cpp)
add_executable(ac_demo ac_demo.cpp)
add_executable(fullbridge_demo fullbridge_demo.cpp)
add_executable(vienna_demo vienna_demo.cpp)
add_executable(dab_demo dab_demo.cpp)
add_executable(llc_demo llc_demo.cpp)
add_executable(pmsm_demo pmsm_demo.cpp)
add_executable(induction_demo induction_demo.cpp)
add_executable(sweep_demo sweep_demo.cpp)

foreach(t IN LISTS PE_EXAMPLES)
  target_link_libraries(${t} PRIVATE power_engine power_engine_warnings)
endforeach()
