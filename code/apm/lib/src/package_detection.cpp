#include "../include/package_detection.h"
#include "../include/package_detection_internal.h"
#include "../include/image_processing.h"
#include <cmath>

using namespace automatic_package_measuring::internal;

namespace automatic_package_measuring {

void FindPackage(const cv::Mat& image, const cv::Mat& edges, std::vector<cv::Point>& package_out) {

	cv::Mat edges_cpy = edges.clone();
	std::vector<std::vector<cv::Point>> contours;
	FindContours(edges_cpy, true, contours);
	PruneShortContours(contours, MIN_PACKAGE_CONTOUR_LENGTH);
	PrunePeripheralContours(contours, image.size());

	if (contours.empty())
		return;

	cv::Mat contours_mat = cv::Mat(image.size(), CV_8UC1, cv::Scalar(0));
	cv::drawContours(contours_mat, contours, -1, cv::Scalar(255));

	std::vector<cv::Vec4i> lines;
	DetectLines(contours_mat, lines);

	double avg_dimension = (image.rows + image.cols) / 2;

	std::vector<cv::Point2i> corners;
	FindCorners(lines, corners, MAX_ANGLE_DIFF, avg_dimension * MAX_DIST);
	if (corners.empty())
		return;

	std::vector<cv::Point> hull;
	cv::convexHull(corners, hull);

	FindConvexPolygon(hull, package_out);
}

namespace internal {

void FindCorners(const std::vector<cv::Vec4i>& lines, std::vector<cv::Point>& corners,
		double max_angle_diff_in_degrees, double max_line_dist) {
	for (int i = 0; i < lines.size(); ++i) {
		for (int j = i + 1; j < lines.size(); ++j) {
			cv::Point2f l1_start = cv::Point2f(lines[i][0], lines[i][1]);
			cv::Point2f l1_end = cv::Point2f(lines[i][2], lines[i][3]);

			cv::Point2f l2_start = cv::Point2f(lines[j][0], lines[j][1]);
			cv::Point2f l2_end = cv::Point2f(lines[j][2], lines[j][3]);

			cv::Point2f intersection;
			if (LineIntersection(l1_start, l1_end, l2_start, l2_end, intersection)) {

				cv::Point2f l1 = (l1_end - l1_start);
				cv::Point2f l2 = (l2_end - l2_start);

				double angle = std::acos(l1.dot(l2) / (cv::norm(l1) * cv::norm(l2)));
				double min_angle = M_PI / 2 - max_angle_diff_in_degrees * M_PI / 180.0;
				double max_angle = M_PI / 2 + max_angle_diff_in_degrees * M_PI / 180.0;
				if (angle >= min_angle && angle <= max_angle) {

					double l1_start_dist = std::min(cv::norm(l1_start - l2_start),
							cv::norm(l1_start - l2_end));
					double l1_end_dist = std::min(cv::norm(l1_end - l2_start), cv::norm(l1_end - l2_end));
					double dist = std::min(l1_start_dist, l1_end_dist);
					if (dist <= max_line_dist) {
						corners.push_back(intersection);
					}

				}

			}
		}
	}
}

bool LineIntersection(cv::Point2f l1_start, cv::Point2f l1_end, cv::Point2f l2_start, cv::Point2f l2_end,
		cv::Point2f &intersection) {

	cv::Point2f x = l2_start - l1_start;
	cv::Point2f d1 = l1_end - l1_start;
	cv::Point2f d2 = l2_end - l2_start;

	float cross = d1.x * d2.y - d1.y * d2.x;
	if (abs(cross) < /*EPS*/1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	intersection = l1_start + d1 * t1;
	return true;
}

double MAX_ANGLE_DIFF = 75.0;
double MAX_DIST = 0.1;
double MIN_PACKAGE_CONTOUR_LENGTH = 200.0;

} /* namespace internal */

} /* namespace automatic_package_measuring */
