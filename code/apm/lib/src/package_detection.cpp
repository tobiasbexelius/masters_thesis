#include "../include/package_detection.h"
#include "../include/package_detection_internal.h"
#include "../include/image_processing.h"
#include <limits>
#include <cmath>
#include <set>
#include <algorithm>
#include <queue>

using namespace automatic_package_measuring::internal;
namespace automatic_package_measuring {
std::vector<cv::Point2f> FindPackage(const cv::Mat& image, const cv::Mat& edges,
		std::vector<cv::Point2f>& reference_object) {
	cv::Mat edges_cpy = edges.clone();
	std::vector<std::vector<cv::Point>> contours;
	FindContours(edges_cpy, true, contours);

	double min_image_dimension = std::min(image.size().width, image.size().height);
	PruneShortContours(contours, min_image_dimension * MIN_PACKAGE_CONTOUR_LENGTH);

	PrunePeripheralContours(contours, image.size());

	if (contours.empty())
		return std::vector<cv::Point2f>();
	cv::Mat contours_mat = cv::Mat(image.size(), CV_8UC1, cv::Scalar(0));
	cv::drawContours(contours_mat, contours, -1, cv::Scalar(255));

	std::vector<cv::Vec4i> lines;
	DetectLines(contours_mat, lines);

	std::vector<std::vector<cv::Point2f>> packages = FindPackages(lines, reference_object, image.size());
	double max_score = -std::numeric_limits<double>::max();
	std::vector<cv::Point2f> max_package;
	for (auto it = packages.begin(); it != packages.end(); ++it) {
		double score = RatePackage(lines, *it);
		if (score > max_score) {
			max_score = score;
			max_package = *it;
		}

	}
	if (max_score < MIN_ACCEPTED_SCORE)
		return std::vector<cv::Point2f>();

	return max_package;

}

namespace internal {

double RatePackage(std::vector<cv::Vec4i>& lines, std::vector<cv::Point2f>& package) {

	double angle_score = 100.0;
	double length_score = 100.0;
	int size = package.size();
	cv::Point prev = package[0];
	for (int i = 1; i < size / 2 + 1; ++i) {
		cv::Point cur = package[i];
		cv::Point opp1 = package[(i + 2) % size];
		cv::Point opp2 = package[(i + 3) % size];

		double length1 = cv::norm(cur - prev);
		double length2 = cv::norm(opp2 - opp1);
		double len_diff = std::min(length1, length2) / std::max(length1, length2);

		length_score *= len_diff;

		double angle = LineSegmentAngle(cv::Vec4i(prev.x, prev.y, cur.x, cur.y),
				cv::Vec4i(opp1.x, opp1.y, opp2.x, opp2.y));

		angle_score *= (1.0 - angle / 90.0);

		prev = cur;
	}
	if (angle_score < MIN_ACCEPTED_SUBSCORE || length_score < MIN_ACCEPTED_SUBSCORE)
		return 0;

	return (angle_score + length_score) / 2;
}

std::vector<std::vector<cv::Point2f>> FindPackages(const std::vector<cv::Vec4i>& lines,
		const std::vector<cv::Point2f>& reference_object, const cv::Size& image_size) {

	std::vector<std::tuple<int, int>> line_pairs;
	int min_image_dimension = std::min(image_size.width, image_size.height);
	FindParallelLines(lines, MIN_PARALLEL_LINE_DIST * min_image_dimension, line_pairs);

	std::vector<std::vector<cv::Point2f>> packages;
	if (line_pairs.size() < 3)
		return packages;

	if (line_pairs.size() > MAX_LINE_PAIRS) {
		std::cout << "Too many line pairs: " << line_pairs.size() << std::endl;
		return std::vector<std::vector<cv::Point2f>>();
	}

	for (int i = 0; i < line_pairs.size(); ++i) {
		for (int j = i + 1; j < line_pairs.size(); ++j) {
			for (int k = j + 1; k < line_pairs.size(); ++k) {
				std::vector<int> line_indices = GetLinesInPairs(line_pairs, { i, j, k });
				if (line_indices.empty())
					continue;

				std::vector<cv::Point2f> package;
				bool isPackageValid = TryToCreatePolygon(lines, line_indices, image_size, package);

				if (!isPackageValid)
					continue;
//				if (!reference_object.empty() && !EnclosesContour(package, reference_object))
//					continue;
				packages.push_back(package);
			}
		}
	}
	return packages;
}

std::vector<int> GetLinesInPairs(const std::vector<std::tuple<int, int>>& line_pairs,
		std::vector<int> indices) {
	std::vector<int> lines;
	for (int i : indices) {
		int line = std::get<0>(line_pairs[i]);

		if (std::find(lines.begin(), lines.end(), line) == lines.end()) // same line twice - not allowed
			lines.push_back(line);
		else
			return std::vector<int>();

		line = std::get<1>(line_pairs[i]);

		if (std::find(lines.begin(), lines.end(), line) == lines.end())
			lines.push_back(line);
		else
			return std::vector<int>();

	}
	return lines;
}

bool EnclosesContour(const std::vector<cv::Point2f>& enclosing_contour,
		const std::vector<cv::Point2f>& enclosed_contour) {
	for (auto it = enclosed_contour.begin(); it != enclosed_contour.end(); ++it) {
		double is_enclosed = cv::pointPolygonTest(enclosing_contour, *it, false);
		if (is_enclosed < 0)
			return false;
	}
	return true;
}

bool TryToCreatePolygon(const std::vector<cv::Vec4i>& lines, const std::vector<int> indices,
		const cv::Size& image_size, std::vector<cv::Point2f>& package) {

	int min_image_dimension = std::min(image_size.width, image_size.height);

	std::vector<cv::Point2f> corners = FindCorners(lines, indices, image_size,
			MIN_CORNER_DIST * min_image_dimension);

	if (corners.size() != indices.size())
		return false;

	std::vector<int> hull;
	cv::convexHull(corners, hull, false); // Y axis points down, this function assumes the opposite. Thus, orientation is reverted.

	if (hull.size() != indices.size())
		return false;

	for (int i = 0; i < hull.size(); ++i) {
		package.push_back(corners[hull[i]]);
	}

	return true;
}

std::vector<cv::Point2f> FindCorners(const std::vector<cv::Vec4i>& lines,
		const std::vector<int>& line_indices, const cv::Size& image_size, const double min_corner_dist) {
	std::vector<cv::Point2f> intersections;

	for (auto l1 = line_indices.begin(); l1 != line_indices.end(); ++l1) {
		cv::Vec4i line1 = lines[*l1];
		cv::Point2f p1 = cv::Point2f(line1[0], line1[1]);
		cv::Point2f p2 = cv::Point2f(line1[2], line1[3]);
		double p1_min_dist = std::numeric_limits<int>::max();
		int p1_neighbour = -1;
		cv::Point2f p1_intersection;
		double p2_min_dist = std::numeric_limits<int>::max();
		int p2_neighbour = -1;
		cv::Point2f p2_intersection;
		for (auto l2 = line_indices.begin(); l2 != line_indices.end(); ++l2) {

			if (l1 == l2)
				continue;

			cv::Vec4i line2 = lines[*l2];
			cv::Point2f intersection;

			if (!FindIntersection(line1, line2, intersection))
				continue;

			if (intersection.x < 0 || intersection.y < 0 || intersection.x > image_size.width
					|| intersection.y > image_size.height)
				continue;

			double p1_dist = EuclideanDistance(p1, intersection);
			double p2_dist = EuclideanDistance(p2, intersection);

			if (p1_dist < p2_dist && p1_dist < p1_min_dist) {
				p1_min_dist = p1_dist;
				p1_neighbour = *l2;
				p1_intersection = intersection;
			}

			if (p2_dist < p1_dist && p2_dist < p2_min_dist) {
				p2_min_dist = p2_dist;
				p2_neighbour = *l2;
				p2_intersection = intersection;
			}
		}

		if (p1_neighbour == -1 || p2_neighbour == -1) {
			return std::vector<cv::Point2f>();
		}

		if (line_indices.size() == 6) { // TODO This is a disgrace, fix
			int l1_pair = (find(line_indices.begin(), line_indices.end(), *l1) - line_indices.begin()) / 2;
			int n1_pair =
					(find(line_indices.begin(), line_indices.end(), p1_neighbour) - line_indices.begin()) / 2;
			int n2_pair =
					(find(line_indices.begin(), line_indices.end(), p2_neighbour) - line_indices.begin()) / 2;
			if (l1_pair == n1_pair || l1_pair == n2_pair || n1_pair == n2_pair) // TODO only true if 3 sides are seen...
				return std::vector<cv::Point2f>();
		}

		bool contains_i1 = false, contains_i2 = false;

		for (auto intersection : intersections) {
			if (EuclideanDistance(intersection, p1_intersection) < min_corner_dist)
				contains_i1 = true;
			else if (EuclideanDistance(intersection, p2_intersection) < min_corner_dist)
				contains_i2 = true;
		}
		if (!contains_i1)
			intersections.push_back(p1_intersection);

		if (!contains_i2)
			intersections.push_back(p2_intersection);
	}

	return intersections;

}

double LineSegmentDistance(const cv::Vec4i& line1, const cv::Vec4i& line2) { // Only checks distance between end points
	cv::Point2f o1 = cv::Point2f(line1[0], line1[1]);
	cv::Point2f e1 = cv::Point2f(line1[2], line1[3]);
	cv::Point2f o2 = cv::Point2f(line2[0], line2[1]);
	cv::Point2f e2 = cv::Point2f(line2[2], line2[3]);
	double o1_dist = std::min(cv::norm(o1 - o2), cv::norm(o1 - e2));
	double e1_dist = std::min(cv::norm(e1 - o2), cv::norm(e1 - e2));
	return std::min(o1_dist, e1_dist);
}

void FindParallelLines(const std::vector<cv::Vec4i>& lines, const double min_line_dist,
		std::vector<std::tuple<int, int>>& parallel_line_pairs) {
	for (int i = 0; i < lines.size(); ++i) {
		for (int j = i + 1; j < lines.size(); ++j) {
			if (LineSegmentAngle(lines[i], lines[j]) < MAX_PARALLEL_LINE_ANGLE
					&& LineSegmentDistance(lines[i], lines[j]) > min_line_dist) {
				parallel_line_pairs.push_back(std::tuple<int, int>(i, j));
			}
		}
	}
}

double LineSegmentAngle(const cv::Vec4i& line1, const cv::Vec4i& line2) {
	cv::Point2f l1 = (cv::Point2f(line1[0], line1[1]) - cv::Point2f(line1[2], line1[3]));
	cv::Point2f l2 = (cv::Point2f(line2[0], line2[1]) - cv::Point2f(line2[2], line2[3]));
	double angle = std::acos(l1.dot(l2) / (cv::norm(l1) * cv::norm(l2))) * 180.0 / CV_PI;
	if (angle > 90.0)
		angle = 180.0 - angle;
	return angle;

}

bool FindIntersection(const cv::Vec4i& line1, const cv::Vec4i& line2, cv::Point2f& intersection) {
	cv::Point2f o1 = cv::Point2f(line1[0], line1[1]);
	cv::Point2f e1 = cv::Point2f(line1[2], line1[3]);
	cv::Point2f o2 = cv::Point2f(line2[0], line2[1]);
	cv::Point2f e2 = cv::Point2f(line2[2], line2[3]);

	cv::Point2f x = o2 - o1;
	cv::Point2f d1 = e1 - o1;
	cv::Point2f d2 = e2 - o2;

	double cross = d1.x * d2.y - d1.y * d2.x;
	if (std::abs(cross) < 1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	intersection = o1 + d1 * t1;
	intersection.x = intersection.x + 0.5;
	intersection.y = intersection.y + 0.5;
	return true;
}

double EuclideanDistance(cv::Point2f& p1, cv::Point2f& p2) {
	return cv::norm(p1 - p2);
}

// TODO make const when tuning is finished
double MIN_CORNER_DIST = 0.02; // of smallest image axis
double MIN_PARALLEL_LINE_DIST = 0.05;
double MIN_ACCEPTED_SCORE = 50.0;
double MAX_PARALLEL_LINE_ANGLE = 30.0;
double MIN_PACKAGE_CONTOUR_LENGTH = 0.2;
int MAX_LINE_PAIRS = 100;
double MIN_ACCEPTED_SUBSCORE = 20.0;

} /* namespace internal */

} /* namespace automatic_package_measuring */
