cmake_minimum_required(VERSION 2.6)

cmake_policy(SET CMP0042 NEW) # CMake 3.0

ADD_DEFINITIONS(
    -std=c++11
    -DDEBUG
)

project(apm_lib)

include_directories(lib/include)
include_directories(lib/include/external)

add_subdirectory(lib)

option(GUI "build gui" ON)
option (TESTS "build tests" ON)

if (TESTS)
  add_subdirectory(test)
endif()
if (GUI)
	add_subdirectory(gui)
endif()