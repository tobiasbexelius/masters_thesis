file(GLOB_RECURSE apm_test_source_files src/*)
add_executable(apm_test ${apm_test_source_files})

add_subdirectory(include/jsoncpp)
target_link_libraries (apm_test jsoncpp)

target_link_libraries(apm_test apm_lib)

find_package( OpenCV REQUIRED ts)
target_link_libraries( apm_test ${OpenCV_LIBS} )

