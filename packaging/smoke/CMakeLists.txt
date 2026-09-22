cmake_minimum_required(VERSION 3.21)
project(pe_consumer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Install-tree smoke test: consumes the INSTALLED power_engine package,
# not the source tree. Configure with
#   -DCMAKE_PREFIX_PATH=<install-prefix>
find_package(power_engine 0.1 CONFIG REQUIRED)

add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE PowerEngine::power_engine)
