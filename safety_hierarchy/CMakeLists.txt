cmake_minimum_required(VERSION 3.16)
project(safety_hierarchy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Threads REQUIRED)

add_library(sh_core
  src/params.cpp
  src/map.cpp
  src/obstacles.cpp
  src/sensor.cpp
  src/observer.cpp
  src/nf1.cpp
  src/distance_field.cpp
  src/costmap.cpp
  src/planner.cpp
  src/simulator.cpp
  src/stats.cpp
  src/image.cpp
)
target_include_directories(sh_core PUBLIC include)
target_compile_definitions(sh_core PUBLIC SH_MAPS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/maps")

foreach(app sh_run sh_experiments sh_tables)
  add_executable(${app} apps/${app}.cpp)
  target_link_libraries(${app} PRIVATE sh_core Threads::Threads)
endforeach()

enable_testing()
add_executable(sh_tests
  tests/test_main.cpp
  tests/test_costmap.cpp
  tests/test_nf1_field.cpp
  tests/test_sensor.cpp
  tests/test_planner.cpp
  tests/test_stats.cpp
)
target_link_libraries(sh_tests PRIVATE sh_core)
add_test(NAME sh_tests COMMAND sh_tests)
