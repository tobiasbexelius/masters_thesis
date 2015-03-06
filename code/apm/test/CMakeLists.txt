find_package( OpenCV REQUIRED ts)
file(GLOB_RECURSE apm_test_source_files src/*)
file(GLOB_RECURSE apm_unit_test_source_files src_gtest/*)
add_executable(apm_test ${apm_test_source_files})
add_executable(apm_unit_test ${apm_unit_test_source_files})

add_subdirectory(include/external/jsoncpp)
add_subdirectory(include/external/gtest-1.7.0)
include_directories(include/external/gtest-1.7.0/include)
include_directories(include/external/jsoncpp)

target_link_libraries (apm_test jsoncpp)
target_link_libraries (apm_test apm_lib)
target_link_libraries( apm_test ${OpenCV_LIBS} )

target_link_libraries (apm_unit_test jsoncpp)
target_link_libraries (apm_unit_test gtest)
target_link_libraries (apm_unit_test apm_lib)
target_link_libraries (apm_unit_test ${OpenCV_LIBS} )
