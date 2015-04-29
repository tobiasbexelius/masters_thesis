#include "../include/package_measuring.h"
#include "../include/package_measuring_internal.h"
#include <cmath>
#include <cstdio>

//#include <android/log.h>
//#define LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
//#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
//#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
//#define LOG_TAG "APM"
#define LOGD printf
using namespace automatic_package_measuring::internal;

namespace automatic_package_measuring {

cv::Vec3f MeasurePackage(const cv::Size& img_size, const std::vector<cv::Point2f>& reference_object,
		const cv::Vec2f ref_dimensions, const std::vector<cv::Point2f>& package,
		cv::Vec3i& measured_edges_out) {
	LOGD("REFERENCE OBJ: %f, %f\n", ref_dimensions[0], ref_dimensions[1]);
	if (reference_object.size() != 4 || package.size() != 6)
		return cv::Vec3f();

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
	LOGD("POINTS (%f, %f),(%f, %f),(%f, %f),(%f, %f),(%f, %f),(%f, %f)\n", package[top_left].x,
			package[top_left].y, package[top_center].x, package[top_center].y, package[top_right].x,
			package[top_right].y, package[bottom_left].x, package[bottom_left].y, package[bottom_center].x,
			package[bottom_center].y, package[bottom_right].x, package[bottom_right].y);

	LOGD("CORNERS %d,%d,%d,%d,%d,%d\n", top_left, top_center, top_right, bottom_left, bottom_center,
			bottom_right);

	cv::Point3f top_left_3d = ProjectPlanarImagePointTo3D(package[top_left], homography);
	cv::Point3f top_center_3d = ProjectPlanarImagePointTo3D(package[top_center], homography);
	cv::Point3f top_right_3d = ProjectPlanarImagePointTo3D(package[top_right], homography);

	LOGD("TOP CORNERS (%f,%f),(%f,%f),(%f,%f)\n", top_left_3d.x, top_left_3d.y, top_center_3d.x,
			top_center_3d.y, top_right_3d.x, top_right_3d.y);

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
	double left_height = CalculateHeight(camera_matrix, package[bottom_left], top_left_3d.x, top_left_3d.y,
			left_error);

	double right_error;
	double right_height = CalculateHeight(camera_matrix, package[bottom_right], top_right_3d.x,
			top_right_3d.y, right_error);

	LOGD("HEIGHTS = %f, %f, ERRORS= %f, %f\n", left_height, right_height, left_error, right_error);

	cv::Vec3f result;

	result[0] = cv::norm(top_left_3d - top_center_3d);
	result[1] = cv::norm(top_center_3d - top_right_3d);

	if (left_error < right_error)
		result[2] = std::abs(left_height);
	else
		result[2] = std::abs(right_height);

	measured_edges_out[0] = top_left;
	measured_edges_out[1] = top_center;
	measured_edges_out[2] = top_right;

	LOGD("RESULT= %f, %f, %f\n", result[0], result[1], result[2]);

	return result;
}

cv::Vec3f UncalibMeasurePackage(const cv::Size& img_size, const std::vector<cv::Point2f>& reference_object,
		const cv::Vec2f ref_dimensions, const std::vector<cv::Point2f>& package,
		cv::Vec3i& measured_edges_out) {

	if (reference_object.size() != 4 || package.size() != 6)
		return cv::Vec3f();

	std::vector<cv::Point2f> ref_obj_world_2d = GetReferenceObjectCoordinates(reference_object,
			ref_dimensions);

	cv::Mat homography = cv::findHomography(reference_object, ref_obj_world_2d);

	// TODO handle special case where only two sides of the package are visible
	// (top and one side parallel-ish to the camera plane). Then four corners are
	// on the top plane and two on the floor.

	int top_left;
	int top_right;
	int mid_left;
	int mid_right;
	int bottom_left;
	int bottom_right;
	IdentifyPackageCornersHeadOn(package, top_left, top_right, mid_left, mid_right, bottom_left,
			bottom_right);
	LOGD("POINTS (%f, %f),(%f, %f),(%f, %f),(%f, %f),(%f, %f),(%f, %f)\n", package[top_left].x,
			package[top_left].y, package[top_right].x, package[mid_left].x, package[mid_left].y,
			package[mid_right].x, package[mid_right].y, package[top_right].y, package[bottom_left].x,
			package[bottom_left].y, package[bottom_right].x, package[bottom_right].y);

	LOGD("CORNERS %d,%d,%d,%d,%d,%d\n", top_left, top_right, mid_left, mid_right, bottom_left, bottom_right);

	cv::Point3f top_left_3d = ProjectPlanarImagePointTo3D(package[top_left], homography);
	cv::Point3f top_right_3d = ProjectPlanarImagePointTo3D(package[top_right], homography);
	cv::Point3f mid_left_3d = ProjectPlanarImagePointTo3D(package[mid_left], homography);
	cv::Point3f mid_right_3d = ProjectPlanarImagePointTo3D(package[mid_right], homography);

	// calc height

	cv::Vec3f result;

	result[0] = cv::norm(top_left_3d - top_right_3d);
	result[1] = cv::norm(top_left_3d - mid_left_3d);
	result[2] = 0;

	measured_edges_out[0] = top_left;
	measured_edges_out[1] = mid_left;
	measured_edges_out[2] = bottom_left;

	LOGD("RESULT= %f, %f, %f\n", result[0], result[1], result[2]);

	return result;
}

namespace internal {

double CalculateHeight(const cv::Mat_<double>& camera_matrix, const cv::Point2f& image_point,
		const float& world_x, const float& world_y, double& error_out) {

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
	//std::cout << "Blir det fel? Kolla om kalibreringsinställningarna matchar bilden. Idiot." << std::endl;
	cv::Mat_<double> diff = (A * x) - b;
	error_out = std::pow(diff[0][0], 2) + std::pow(diff[1][0], 2) + std::pow(diff[2][0], 2);

	////////// Alternativt sätt

	cv::Mat_<double> p;
	camera_matrix.copyTo(p);
	p /= p[2][3];

	cv::Mat_<double> Aalt = (cv::Mat_<double>(2, 1) << image_point.x * p[2][2] - p[0][2], image_point.y
			* p[2][2] - p[1][2]);
	cv::Mat_<double> balt = (cv::Mat_<double>(2, 1)
			<< world_x * (p[0][0] - image_point.x * p[2][0]) + world_y * (p[0][1] - image_point.x * p[2][1])
					+ p[0][3] - image_point.x, world_x * (p[1][0] - image_point.y * p[2][0])
			+ world_y * (p[1][1] - image_point.y * p[2][1]) + p[1][3] - image_point.y);
	cv::Mat_<double> xalt;
	cv::solve(Aalt, balt, xalt, cv::DECOMP_SVD);
	cv::Mat_<double> diffalt = (Aalt * xalt) - balt;
	double error_out_alt = std::pow(diffalt[0][0], 2) + std::pow(diffalt[1][0], 2);
	std::cout << "======================= COMPARISON BETWEEN OLD AND NEW METHOD ======================="
			<< std::endl;
	std::cout << "OLD Z = " << x[1][0] << ", ERROR = " << error_out << std::endl;
	std::cout << "NEW Z = " << xalt[0][0] << ", ERROR = " << error_out_alt << std::endl;

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

void IdentifyPackageCornersHeadOn(const std::vector<cv::Point2f>& corners, int& top_left, int& top_right,
		int& mid_left, int& mid_right, int& bottom_left, int& bottom_right) { // assumes corners is in clockwise order
	int top_corner = -1;
	int top_corner_y = std::numeric_limits<int>::max(); // y axis points down so top corner has smallest y.

	for (int i = 0; i < corners.size(); ++i) {
		if (corners[i].y < top_corner_y) {
			top_corner = i;
			top_corner_y = corners[i].y;
		}
	}

	cv::Point2f top = corners[top_corner];
	cv::Point2f left = corners[(top_corner - 1 + 6) % 6];
	cv::Point2f right = corners[(top_corner + 1) % 6];

	double left_dx = left.x - top.x;
	double right_dx = right.x - top.x;
	if (left_dx == 0)
		top_left = top_corner;
	else if (right_dx == 0)
		top_left = (top_corner - 1 + 6) % 6;
	else {
		double left_slope = std::abs((left.y - top.y) / (left.x - left.y));
		double right_slope = std::abs((right.y - top.y) / (right.x - left.y));
		if (left_slope > right_slope)
			top_left = top_corner;
		else
			top_left = (top_corner - 1 + 6) % 6;
	}

	top_right = (top_left + 1) % 6;
	mid_right = (top_left + 2) % 6;
	bottom_right = (top_left + 3) % 6;
	bottom_left = (top_left + 4) % 6;
	mid_left = (top_left + 5) % 6;

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

std::vector<cv::Point2f> FindVanishingPoints(std::vector<cv::Point2f>& package) {
	std::vector<cv::Point2f> vanishing_points;

	for(int i = 0; i< package.size(); ++i) {
		cv::Vec4i line = cv::Vec4i(package[i].x, package[i].y, package[(i+1)%6].x,package[(i+1)%6].y); // TODO behövs float points???
		int opp = i+3;
		cv::Vec4i opposing = cv::Vec4i(package[opp].x, package[opp].y, package[(opp+1)%6].x, package[(opp+1)%6].y);
		cv::Point2f intersection;
		if(!FindIntersection(line, opp, intersection))
			return std::vector<cv::Point2f>();
		vanishing_points.push_back(intersection);
	}
	return vanishing_points;
}

/////////////// GALAXY S6 //////////////////////////////////

/////////////// FOR ASPECT RATIO 16:9 //////////////////////

//const int calib_width = 720;
//const int calib_height = 1280;
//const cv::Mat intrinsic_parameters =
//		(cv::Mat_<double>(3, 3) << 9.4476124447804500e+02, 0, 3.6023041906015595e+02, 0, 9.4988631379427648e+02, 6.1579670879484991e+02, 0, 0, 1);
//const cv::Mat distortion_coeffs =
//		(cv::Mat_<double>(5, 1) << 9.0884505869291463e-02, 9.9527802556862555e-01, -4.2888133638235562e-03, 1.2852458935930279e-03, -5.1800825723712123e+00);

/////////////// GALXY S3 ///////////////////////////////////

/////////////// FOR ASPECT RATIO 16:9 //////////////////////

//const int calib_width = 720;
//const int calib_height = 1280;
//const cv::Mat intrinsic_parameters =
//		(cv::Mat_<double>(3, 3) << 1.0383022526373247e+03, 0, 6.4815656332212768e+02, 0, 1.0312714841510390e+03, 3.3912005664156112e+02, 0, 0, 1);
//const cv::Mat distortion_coeffs =
//		(cv::Mat_<double>(5, 1) << -5.6081669213434132e-02, 1.9348361811630961e+00, -1.6428716030314656e-03, -8.5823452014998358e-03, -7.6018511226007863e+00);

/////////////// FOR ASPECT RATIO 4:3 ///////////////////////

// const int calib_width = 2448;
// const int calib_height = 3264;
// const cv::Mat intrinsic_parameters =
// (cv::Mat_<double>(3, 3) << 2700.6936935016442, 0, 1631.5000000000000, 0, 2700.6936935016442, 1223.5000000000000, 0, 0, 1);
// const cv::Mat distortion_coeffs =
// (cv::Mat_<double>(5, 1) << -0.023547506752116257e-02, 1.7597479866273940, 0, 0, -8.0444205336834855);

const int calib_width = 768;
const int calib_height = 1024;
const cv::Mat intrinsic_parameters =
		(cv::Mat_<double>(3, 3) << 8.4792405910557568e+02, 0, 3.8350000000000000e+02, 0, 8.4792405910557568e+02, 5.1150000000000000e+02, 0, 0, 1);
const cv::Mat distortion_coeffs =
		(cv::Mat_<double>(5, 1) << 3.3562504822751478e-02, 5.3683800058354936e-01, 0, 0, -1.7276201066920471e+00);

}
/* namespace internal */

} /* namespace automatic_package_measuring */
