find_package( OpenCV REQUIRED ts)
file(GLOB calibration_src calibrate/src/calibration.cpp)
add_executable(calibration ${calibration_src})

file(GLOB undistort_test_src undistort_test/src/*.cpp)
add_executable(undistort_test ${undistort_test_src})

file(GLOB imagelist_creator_src calibrate/src/imagelist_creator.cpp)
add_executable(imagelist_creator ${imagelist_creator_src})

find_package(OpenCV REQUIRED core)
find_package(OpenCV REQUIRED imgproc)
find_package(OpenCV REQUIRED highgui)
find_package(OpenCV REQUIRED flann)
find_package(OpenCV REQUIRED calib3d)

target_link_libraries(calibration ${OpenCV_LIBS} )
target_link_libraries (undistort_test ${OpenCV_LIBS} )
target_link_libraries (imagelist_creator ${OpenCV_LIBS} )