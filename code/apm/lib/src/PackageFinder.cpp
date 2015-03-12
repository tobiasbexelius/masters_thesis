#include "PackageFinder.h"
#include "ContourExtractor.h"
#include "HoughLineTransform.h"
#include "../include/PolygonFinder.h"
#include <cmath>
namespace automatic_package_measuring {

PackageFinder::PackageFinder() {

}

PackageFinder::~PackageFinder() {
}

std::vector<cv::Point> PackageFinder::findObject(cv::Mat image,
		std::vector<std::vector<cv::Point> > contours) {

	cv::Mat contours_mat = cv::Mat(image.size(), CV_8UC1, cv::Scalar(0));
	cv::drawContours(contours_mat, contours, -1, cv::Scalar(255));

	HoughLineTransform hough;
	std::vector<cv::Vec4i> lines;

	lines = hough.detectLines(contours_mat);

	std::vector<cv::Point2i> corners;

	double avg_dimension = (image.rows + image.cols) / 2;

	findCorners(lines, corners, MAX_ANGLE_DIFF, avg_dimension * MAX_DIST);
	std::vector<cv::Point> hull;
	if (corners.empty())
		return std::vector<cv::Point>();
	cv::convexHull(corners, hull);
	std::vector<std::vector<cv::Point>> poly_vec;
	poly_vec.push_back(hull);
	PolygonFinder polygon_finder;
	std::vector<std::vector<cv::Point>> package = polygon_finder.findPolygons(poly_vec);
	return package[0];
}

bool PackageFinder::lineIntersection(cv::Point2f l1_start, cv::Point2f l1_end, cv::Point2f l2_start,
		cv::Point2f l2_end, cv::Point2f &intersection) {

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

void PackageFinder::findCorners(const std::vector<cv::Vec4i>& lines, std::vector<cv::Point2i>& corners,
		double max_angle_diff_in_degrees, double max_line_dist) {

	for (int i = 0; i < lines.size() - 1; ++i) {
		for (int j = i + 1; j < lines.size(); ++j) {

			cv::Point2f l1_start = cv::Point2f(lines[i][0], lines[i][1]);
			cv::Point2f l1_end = cv::Point2f(lines[i][2], lines[i][3]);

			cv::Point2f l2_start = cv::Point2f(lines[j][0], lines[j][1]);
			cv::Point2f l2_end = cv::Point2f(lines[j][2], lines[j][3]);

			cv::Point2f intersection;
			if (lineIntersection(l1_start, l1_end, l2_start, l2_end, intersection)) {

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
//						std::cout << "Intersection between i=" << i << ":" << l1_start << " -> " << l1_end
//								<< " and j=" << j << ":" << l2_start << " -> " << l2_end << " at "
//								<< intersection << " angle is " << (angle * 180 / M_PI)  << " and dist is " << dist << " (max " << max_line_dist << ")"<<  std::endl;
						corners.push_back(intersection);
					}

				}

			}
		}
	}
}
} /* namespace automatic_package_measuring */
