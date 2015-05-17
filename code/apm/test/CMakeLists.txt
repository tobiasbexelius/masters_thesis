find_package( OpenCV REQUIRED ts)
file(GLOB apm_test_runner_src src/test_runner/*.cpp src/*.cpp)
add_executable(test_runner ${apm_test_runner_src})

file(GLOB apm_test_creator_src src/test_creator/*.cpp src/*.cpp)
add_executable(test_creator ${apm_test_creator_src})

file(GLOB_RECURSE apm_gtest_src src/gtest/*.cpp)
add_executable(apm_gtest ${apm_gtest_src})

file(GLOB apm_test_printer_src src/test_printer/*.cpp src/*.cpp)
add_executable(test_printer ${apm_test_printer_src})

add_subdirectory(include/external/jsoncpp)
add_subdirectory(include/external/gtest-1.7.0)
include_directories(include/external/gtest-1.7.0/include)
include_directories(include/external/jsoncpp)

target_link_libraries (test_runner jsoncpp)
target_link_libraries (test_runner apm_lib)
target_link_libraries (test_runner ${OpenCV_LIBS} )

target_link_libraries (test_creator jsoncpp)
target_link_libraries (test_creator apm_lib)
target_link_libraries (test_creator ${OpenCV_LIBS} )

target_link_libraries (apm_gtest jsoncpp)
target_link_libraries (apm_gtest gtest)
target_link_libraries (apm_gtest apm_lib)
target_link_libraries (apm_gtest ${OpenCV_LIBS} )

target_link_libraries (test_printer jsoncpp)
target_link_libraries (test_printer apm_lib)
target_link_libraries (test_printer ${OpenCV_LIBS} )

