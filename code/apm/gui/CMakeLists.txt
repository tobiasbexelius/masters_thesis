file(GLOB apm_test_gui_source_files *.cpp)
add_executable(apm_test_gui ${apm_test_gui_source_files})

target_link_libraries(apm_test_gui apm_lib)

target_link_libraries( apm_test_gui ${OpenCV_LIBS} )

