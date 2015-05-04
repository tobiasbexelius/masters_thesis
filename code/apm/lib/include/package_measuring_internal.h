#ifndef LIB_INCLUDE_PACKAGE_MEASURING_INTERNAL_H_
#define LIB_INCLUDE_PACKAGE_MEASURING_INTERNAL_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

namespace internal {

std::vector<cv::Point2f> GetReferenceObjectCoordinates(const std::vector<cv::Point2f>& image_coordinates,
		const cv::Vec2f& dimensions);
int GetLongestEdgeEndCorner(const std::vector<cv::Point2f>& corners);
cv::Mat GetIntrinsicMatrix(int target_width, int target_height);
cv::Mat_<double> GetHomography(const cv::Mat_<double>& rotation, const cv::Mat_<double>& translation);
cv::Point3f ProjectPlanarImagePointTo3D(const cv::Point2f& image_point, const cv::Mat_<double>& homography);
void IdentifyPackageCorners(const std::vector<cv::Point2f>& package, int& top_left, int& top_center,
		int& top_right, int& bottom_left, int& bottom_center, int& bottom_right);
double CalculateHeight(const cv::Mat_<double>& camera_matrix, const cv::Point2f& image_point,
		const float& world_x, const float& world_y, double& error_out);

void IdentifyPackageCornersHeadOn(const std::vector<cv::Point2f>& corners, int& top_left, int& top_right,
		int& mid_left, int& mid_right, int& bottom_left, int& bottom_right);

std::vector<cv::Point2f> FindVanishingPoints(const std::vector<cv::Point2f>& package);
cv::Mat_<float> AutoCalibrate(const cv::Mat_<double>& homography, const std::vector<cv::Point2f>& package);
cv::Mat_<double> CreateConstraint(const cv::Vec3f& u, const cv::Vec3f& v);
cv::Mat CholeskyDecomposition(cv::Mat mat);

cv::Mat_<float> FindCameraPose(const std::vector<cv::Point2f>& reference_object,
		const std::vector<cv::Point2f>& ref_obj_world, const cv::Mat_<float>& intrinsic_matrix,
		const cv::Mat_<float>& distortion_coeffs);
extern const int calib_width;
extern const int calib_height;
extern const cv::Mat calib_intrinsic;
extern const cv::Mat calib_distortion;

} /* namespace internal */

} /* namespace automatic_package_measuring */

#endif /* LIB_INCLUDE_PACKAGE_MEASURING_INTERNAL_H_ */
