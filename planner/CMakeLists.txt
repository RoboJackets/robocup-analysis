include_directories ("${CMAKE_CURRENT_SOURCE_DIR}/qpOASES/include")
message (STATUS "${CMAKE_CURRENT_SOURCE_DIR}/qpOASES/include")
add_subdirectory (qpOASES)
cmake_minimum_required (VERSION 2.6)

project (planner)

add_executable (planner planner.cpp)
target_link_libraries (planner qpOASES)
