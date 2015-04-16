#include "../include/package_measuring.h"
#include "../include/package_measuring_internal.h"
#include <cmath>
using namespace automatic_package_measuring::internal;

namespace automatic_package_measuring {

cv::Vec3f MeasurePackage(const cv::Size& img_size, const std::vector<cv::Point2f>& reference_object,
		const cv::Vec2f ref_dimensions, const std::vector<cv::Point2f>& package) {

	if (reference_object.size() != 4 || package.size() != 6)
		return cv::Vec3d();

	std::vector<cv::Point2f> ref_obj_world_2d = GetReferenceObjectCoordinates(reference_object,
			ref_dimensions);

	cv::Mat homography = cv::findHomography(reference_object, ref_obj_world_2d);

	// TODO handle special case where only two sides of the package are visible
	// (top and one side parallel-ish to the camera plane). Then four corners are
	// on the top plane and two on the floor.
	int top_left;
	int top_center;
	int top_right;
	int bottom_left;
	int bottom_center;
	int bottom_right;
	IdentifyPackageCorners(package, top_left, top_center, top_right, bottom_left, bottom_center,
			bottom_right);

	cv::Point3f top_left_3d = ProjectPlanarImagePointTo3D(package[top_left], homography);
	cv::Point3f top_center_3d = ProjectPlanarImagePointTo3D(package[top_center], homography);
	cv::Point3f top_right_3d = ProjectPlanarImagePointTo3D(package[top_right], homography);

	std::vector<cv::Point3f> ref_obj_world_3d;
	for (auto point : ref_obj_world_2d)
		ref_obj_world_3d.push_back(cv::Point3f(point.x, point.y, 0));

	cv::Mat_<double> rotation_vec;
	cv::Mat_<double> translation;
	cv::solvePnP(ref_obj_world_3d, reference_object, ScaleCameraMatrix(img_size.width, img_size.height),
			distortion_coeffs, rotation_vec, translation);

	cv::Mat_<double> rotation_mat;
	cv::Rodrigues(rotation_vec, rotation_mat);

	cv::Mat_<double> extrinsic;
	cv::hconcat(rotation_mat, translation, extrinsic);
	cv::Mat_<double> camera_matrix = intrinsic_parameters * extrinsic;

	double left_error;
	double left_height = CalculateHeight(camera_matrix, package[bottom_left],
			top_left_3d.x, top_left_3d.y, left_error);
	double right_error;
	double right_height = CalculateHeight(camera_matrix, package[bottom_right],
			top_right_3d.x, top_right_3d.y, right_error);

//	std::cout << "Bottom left = " << left_height << std::endl;
//	std::cout << "Bottom right = " << right_height << std::endl;
	cv::Vec3f result;

	result[0] = cv::norm(top_left_3d-top_center_3d);
	result[1] = cv::norm(top_center_3d-top_right_3d);

	if (left_error < right_error)
		result[2] = std::abs(left_height);
	else
		result[2] = std::abs(right_height);


	std::cout << "RES=" << result << std::endl;
	return result;
}

namespace internal {

double CalculateHeight(const cv::Mat_<double>& camera_matrix,
		const cv::Point2f& image_point, const float& world_x, const float& world_y, double& error_out) {

	const cv::Mat_<double>& C = camera_matrix;
	double a1 = C[0][3] + C[0][0] * world_x + C[0][1] * world_y;
	double a2 = C[1][3] + C[1][0] * world_x + C[1][1] * world_y;
	double a3 = C[2][3] + C[2][0] * world_x + C[2][1] * world_y;

	double b1 = C[0][2];
	double b2 = C[1][2];
	double b3 = C[2][2];

	cv::Mat_<double> A = (cv::Mat_<double>(3, 2) << image_point.x, -b1, image_point.y, -b2, 1, -b3);
	cv::Mat_<double> b = (cv::Mat_<double>(3, 1) << a1, a2, a3);
	cv::Mat_<double> x;
	cv::solve(A, b, x, cv::DECOMP_SVD);
	cv::Mat_<double> diff = (A * x) - b;
	error_out = std::pow(diff[0][0], 2) + std::pow(diff[1][0], 2) + std::pow(diff[2][0], 2);

	//double Z1 = (a3 * image_point.x - a1) / (b1 - b3 * image_point.x);
	//double Z2 = (a3 * image_point.y - a2) / (b2 - b3 * image_point.y);
	return x[1][0];
}

void IdentifyPackageCorners(const std::vector<cv::Point2f>& corners, int& top_left, int& top_center,
		int& top_right, int& bottom_left, int& bottom_center, int& bottom_right) { // assumes corners is in clockwise order
	int top_corner = -1;
	int top_corner_y = std::numeric_limits<int>::max(); // y axis points down so top corner has smallest y.

	for (int i = 0; i < corners.size(); ++i) {
		if (corners[i].y < top_corner_y) {
			top_corner = i;
			top_corner_y = corners[i].y;
		}
	}

	top_center = top_corner;
	top_left = (top_corner - 1 + 6) % 6;
	top_right = (top_corner + 1) % 6;
	bottom_center = (top_corner + 3) % 6;
	bottom_left = (top_corner + 4) % 6;
	bottom_right = (top_corner + 2) % 6;
}

cv::Point3f ProjectPlanarImagePointTo3D(const cv::Point2f& image_point, const cv::Mat_<double>& homography) {
	cv::Mat_<double> point_3d = (cv::Mat_<double>(3, 1) << image_point.x, image_point.y, 1.0);
	point_3d = homography * point_3d;
	point_3d = point_3d / point_3d[0][2];
	cv::Point3f projected_point(point_3d[0][0], point_3d[0][1], point_3d[0][2]);
	return projected_point;
}

cv::Mat_<double> GetHomography(const cv::Mat_<double>& rotation, const cv::Mat_<double>& translation) {
	cv::Mat_<double> homography(3, 3);
	homography[0][0] = rotation[0][0];
	homography[1][0] = rotation[1][0];
	homography[2][0] = rotation[2][0];

	homography[0][1] = rotation[0][1];
	homography[1][1] = rotation[1][1];
	homography[2][1] = rotation[2][1];

	homography[0][2] = translation[0][0];
	homography[1][2] = translation[0][1];
	homography[2][2] = translation[0][2];

	homography = homography / translation[0][2];
	return homography;
}

cv::Mat ScaleCameraMatrix(int target_width, int target_height) {
	cv::Mat scaled_camera_matrix = intrinsic_parameters.clone();
	scaled_camera_matrix.at<double>(0, 0) *= ((double) target_width / calib_width);
	scaled_camera_matrix.at<double>(0, 2) *= ((double) target_width / calib_width);
	scaled_camera_matrix.at<double>(1, 1) *= ((double) target_height / calib_height);
	scaled_camera_matrix.at<double>(1, 2) *= ((double) target_height / calib_height);
	return scaled_camera_matrix;
}

std::vector<cv::Point2f> GetReferenceObjectCoordinates(const std::vector<cv::Point2f>& corners,
		const cv::Vec2f& dimensions) {

	std::vector<cv::Point2f> coordinates;

	int longest_edge = GetLongestEdgeEndCorner(corners);
	double long_edge = std::max(dimensions[0], dimensions[1]);
	double short_edge = std::min(dimensions[0], dimensions[1]);

	coordinates.push_back(cv::Point2f(0, 0));

	if (longest_edge == 1 || longest_edge == 3) {
		coordinates.push_back(cv::Point2f(0, long_edge));
		coordinates.push_back(cv::Point2f(short_edge, long_edge));
		coordinates.push_back(cv::Point2f(short_edge, 0));
	} else {
		coordinates.push_back(cv::Point2f(0, short_edge));
		coordinates.push_back(cv::Point2f(long_edge, short_edge));
		coordinates.push_back(cv::Point2f(long_edge, 0));
	}

	return coordinates;
}

int GetLongestEdgeEndCorner(const std::vector<cv::Point2f>& corners) {
	double max_length = std::numeric_limits<int>::min();
	int max_edge = -1;

	cv::Point first = corners[0];
	cv::Point previous = first;
	for (int i = 1; i < corners.size(); ++i) {
		cv::Point current = corners[i];
		double length = cv::norm(current - previous);
		if (length > max_length) {
			max_length = length;
			max_edge = i;
		}
		previous = current;
	}

	double length = cv::norm(previous - first);
	if (length > max_length) {
		max_length = length;
		max_edge = 0;
	}

	return max_edge;
}
/*
 const int calib_width = 3264;
 const int calib_height = 2448;
 const cv::Mat intrinsic_parameters =
 (cv::Mat_<double>(3, 3) << 2700.6936935016442, 0, 1631.5000000000000, 0, 2700.6936935016442, 1223.5000000000000, 0, 0, 1);
 const cv::Mat distortion_coeffs =
 (cv::Mat_<double>(5, 1) << -0.023547506752116257e-02, 1.7597479866273940, 0, 0, -8.0444205336834855);
 */
const int calib_width = 768;
const int calib_height = 1024;
const cv::Mat intrinsic_parameters =
		(cv::Mat_<double>(3, 3) << 8.4792405910557568e+02, 0, 3.8350000000000000e+02, 0, 8.4792405910557568e+02, 5.1150000000000000e+02, 0, 0, 1);
const cv::Mat distortion_coeffs =
		(cv::Mat_<double>(5, 1) << 3.3562504822751478e-02, 5.3683800058354936e-01, 0, 0, -1.7276201066920471e+00);

} /* namespace internal */

} /* namespace automatic_package_measuring */
