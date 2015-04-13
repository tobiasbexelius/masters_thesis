find_package( OpenCV REQUIRED ts)
file(GLOB calib_camera_src calibrate/src/*.cpp)
add_executable(calib_camera ${calib_camera_src})

file(GLOB undistort_test_src undistort_test/src/*.cpp)
add_executable(undistort_test ${undistort_test_src})

find_package(OpenCV REQUIRED core)
find_package(OpenCV REQUIRED imgproc)
find_package(OpenCV REQUIRED highgui)
find_package(OpenCV REQUIRED flann)
find_package(OpenCV REQUIRED calib3d)

target_link_libraries( calib_camera ${OpenCV_LIBS} )
target_link_libraries (undistort_test ${OpenCV_LIBS} )