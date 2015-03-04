file(GLOB_RECURSE apm_lib_source src/*)
add_library(apm_lib SHARED ${apm_lib_source})

find_package(OpenCV REQUIRED core)
find_package(OpenCV REQUIRED imgproc)
find_package(OpenCV REQUIRED highgui)
target_link_libraries(apm_lib ${OpenCV_LIBS})