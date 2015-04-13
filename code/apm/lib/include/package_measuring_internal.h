#ifndef LIB_INCLUDE_PACKAGE_MEASURING_INTERNAL_H_
#define LIB_INCLUDE_PACKAGE_MEASURING_INTERNAL_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

namespace internal {

std::vector<cv::Point2f> GetReferenceObjectCoordinates(const std::vector<cv::Point2f>& image_coordinates,
		const cv::Vec2f& dimensions);
int GetLongestEdgeEndCorner(const std::vector<cv::Point2f>& corners);
cv::Mat ScaleCameraMatrix(int target_width, int target_height);
cv::Mat_<double> GetHomography(const cv::Mat_<double>& rotation, const cv::Mat_<double>& translation);
cv::Point3f ProjectPlanarImagePointTo3D(const cv::Point2f& image_point, const cv::Mat_<double>& homography);
void IdentifyPackageCorners(const std::vector<cv::Point2f>& package, int& top_left, int& top_center, int& top_right,
		int& bottom_left, int& bottom_center, int& bottom_right);
cv::Point3f FindBottomCornerWorldCoordinates(const cv::Mat_<double>& camera_matrix, const cv::Point2f& image_point,
		const float& world_x, const float& world_y);
extern const int calib_width;
extern const int calib_height;
extern const cv::Mat intrinsic_parameters;
extern const cv::Mat distortion_coeffs;

} /* namespace internal */

} /* namespace automatic_package_measuring */

#endif /* LIB_INCLUDE_PACKAGE_MEASURING_INTERNAL_H_ */
