#include "../include/package_detection.h"
#include "../include/package_detection_internal.h"
#include "../include/image_processing.h"
#include "../include/disjoint_set.h"
#include <limits>
#include <cmath>
#include <set>
#include <algorithm>
#include <queue>

using namespace automatic_package_measuring::internal;

namespace automatic_package_measuring {
cv::Mat img;
std::vector<cv::Point2f> FindPackage(const cv::Mat& image, const cv::Mat& edges,
		std::vector<cv::Point2f>& reference_object) {
	cv::Mat edges_cpy = edges.clone();
	std::vector<std::vector<cv::Point>> contours;
	FindContours(edges_cpy, true, contours);
	PruneShortContours(contours, MIN_PACKAGE_CONTOUR_LENGTH);
	PrunePeripheralContours(contours, image.size());
	if (contours.empty())
		return std::vector<cv::Point2f>();
	img = image;
	cv::Mat contours_mat = cv::Mat(image.size(), CV_8UC1, cv::Scalar(0));
	cv::drawContours(contours_mat, contours, -1, cv::Scalar(255));

	std::vector<cv::Vec4i> lines;
	DetectLines(contours_mat, lines);

	std::vector<Package> packages = FindPackages(lines, reference_object, image.size());
	int max_score = std::numeric_limits<int>::min();
	Package max_package;

	for (auto it = packages.begin(); it != packages.end(); ++it) {
		double score = RatePackage(lines, *it);
		if (score > max_score) {
			max_score = score;
			max_package = *it;
		}

	}

	if (max_score < MIN_ACCEPTED_SCORE)
		return std::vector<cv::Point2f>();

	return max_package.corners;

}

namespace internal {

double RatePackage(std::vector<cv::Vec4i>& lines, Package& package) {

	double angle_score = 1000.0;
	double length_score = 1000.0;
	int size = package.corners.size();
	cv::Point prev = package.corners[0];
	for (int i = 1; i < size / 2 + 1; ++i) {
		cv::Point cur = package.corners[i];
		cv::Point opp1 = package.corners[(i + 2) % size];
		cv::Point opp2 = package.corners[(i + 3) % size];

		double length1 = cv::norm(cur - prev);
		double length2 = cv::norm(opp2 - opp1);

		double len_diff = std::min(length1, length2) / std::max(length1, length2);

		length_score *= len_diff;

		double angle = LineSegmentAngle(cv::Vec4i(prev.x, prev.y, cur.x, cur.y),
				cv::Vec4i(opp1.x, opp1.y, opp2.x, opp2.y));

		angle_score *= (1.0 - angle / 90.0);

		prev = cur;
	}

	return angle_score * length_score;
}

std::vector<Package> FindPackages(const std::vector<cv::Vec4i>& lines,
		const std::vector<cv::Point2f>& reference_object, const cv::Size& image_size) {
	std::vector<Package> packages;
	std::vector<std::tuple<int, int>> line_pairs;
	FindParallelLines(lines, line_pairs);
	if (line_pairs.size() < 3)
		return packages;

	if (line_pairs.size() > 50) {
		std::cout << "Too many line pairs: " << line_pairs.size() << std::endl;
		return std::vector<Package>();
	}

	for (int i = 0; i < line_pairs.size(); ++i) {
		for (int j = i + 1; j < line_pairs.size(); ++j) {
			for (int k = j + 1; k < line_pairs.size(); ++k) {

				std::vector<int> indices = { i, j, k };
				Package package;
				bool isPackageValid = TryToCreatePackage(lines, line_pairs, indices, image_size, package);
				if (!isPackageValid)
					continue;

				//if (!reference_object.empty() && !EnclosesContour(package.corners, reference_object))
				//	continue;

				packages.push_back(package);
			}
		}
	}

	return packages;
}

bool EnclosesContour(std::vector<cv::Point>& enclosing_contour, std::vector<cv::Point>& enclosed_contour) {
	for (auto it = enclosed_contour.begin(); it != enclosed_contour.end(); ++it) {
		double is_enclosed = cv::pointPolygonTest(enclosing_contour, *it, false);
		if (is_enclosed < 0)
			return false;
	}
	return true;
}

bool TryToCreatePackage(const std::vector<cv::Vec4i>& lines,
		const std::vector<std::tuple<int, int>>& line_pairs, const std::vector<int>& line_pair_indices,
		const cv::Size& image_size, Package& package) {

	for (int i = 0; i < line_pair_indices.size(); ++i) {
		package.line_to_pair[std::get<0>(line_pairs[line_pair_indices[i]])] = line_pair_indices[i];
		package.line_to_pair[std::get<1>(line_pairs[line_pair_indices[i]])] = line_pair_indices[i];
	}

	if (package.line_to_pair.size() != 6) // contains doubles
		return false;

	for (auto it = package.line_to_pair.begin(); it != package.line_to_pair.end(); ++it) {
		if (!package.pair_to_lines.count(it->second))
			package.pair_to_lines[it->second] = std::vector<int>();

		package.pair_to_lines[it->second].push_back(it->first);
	}

	// Don't allow adjacent lines to be too parallel
	/*for (auto it = neighbour_list.begin(); it != neighbour_list.end(); ++it) {
	 int line = it->first;
	 int neighbour = it->second[0];
	 if (LineSegmentAngle(lines[line], lines[neighbour]) < MIN_ADJACENT_LINE_ANGLE)
	 return false;

	 neighbour = it->second[1];
	 if (LineSegmentAngle(lines[line], lines[neighbour]) < MIN_ADJACENT_LINE_ANGLE)
	 return false;
	 }*/

	std::vector<int> line_indices;
	for (auto i : line_pair_indices) {
		line_indices.push_back(std::get<0>(line_pairs[i]));
		line_indices.push_back(std::get<1>(line_pairs[i]));
	}
	int min_image_dimension = std::min(image_size.width, image_size.height);

	std::vector<cv::Point2f> corners = FindCorners(lines, line_indices,
			MIN_CORNER_DIST * min_image_dimension);

	if (corners.size() != 6)
		return false;

	std::vector<int> hull;
	cv::convexHull(corners, hull, false); // Y axis points down, this function assumes the opposite. Thus, orientation is reverted.

	if (hull.size() != 6)
		return false;

	for (int i = 0; i < hull.size(); ++i) {
		package.corners.push_back(corners[hull[i]]);
	}

	return true;
}

std::vector<cv::Point2f> FindCorners(const std::vector<cv::Vec4i>& lines,
		const std::vector<int>& line_indices, double min_corner_dist) {
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

double LineSegmentDistance(const cv::Vec4i& line1, const cv::Vec4i& line2) { // TODO comparing end points good enough?
	cv::Point2f o1 = cv::Point2f(line1[0], line1[1]);
	cv::Point2f e1 = cv::Point2f(line1[2], line1[3]);
	cv::Point2f o2 = cv::Point2f(line2[0], line2[1]);
	cv::Point2f e2 = cv::Point2f(line2[2], line2[3]);
	double o1_dist = std::min(cv::norm(o1 - o2), cv::norm(o1 - e2));
	double e1_dist = std::min(cv::norm(e1 - o2), cv::norm(e1 - e2));
	return std::min(o1_dist, e1_dist);
}

void FindParallelLines(const std::vector<cv::Vec4i>& lines,
		std::vector<std::tuple<int, int>>& parallel_line_pairs) {
	for (int i = 0; i < lines.size(); ++i) {
		for (int j = i + 1; j < lines.size(); ++j) {
			if (LineSegmentAngle(lines[i], lines[j]) < MAX_PARALLEL_LINE_ANGLE
					&& LineSegmentDistance(lines[i], lines[j]) > MIN_PARALLEL_LINE_DIST) {
				parallel_line_pairs.push_back(std::tuple<int, int>(i, j));
			}
		}
	}
}

bool HasDuplicates(std::vector<int>& vec) {
	std::set<int> duplicate_check(vec.begin(), vec.end());
	return (duplicate_check.size() != vec.size());
}

double LineSegmentAngle(const cv::Vec4i& line1, const cv::Vec4i& line2) {
	cv::Point2f l1 = (cv::Point2f(line1[0], line1[1]) - cv::Point2f(line1[2], line1[3]));
	cv::Point2f l2 = (cv::Point2f(line2[0], line2[1]) - cv::Point2f(line2[2], line2[3]));
	double angle = std::acos(l1.dot(l2) / (cv::norm(l1) * cv::norm(l2))) * 180.0 / CV_PI;
	if (angle > 90.0)
		angle = 180.0 - angle;
	return angle;

}

void FindConnectedComponents(const std::vector<std::vector<Intersection>>& intersections,
		std::vector<std::set<int>>& connected_components) {

	DisjointSet ds(intersections.size());

	for (int i = 0; i < intersections.size(); ++i) {
		for (int j = 0; j < intersections[i].size(); ++j) {
			//	ds.Union(intersections[i][j].l1, intersections[i][j].l2);
		}
	}
	connected_components = ds.AsSets();

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
	intersection.x = std::round(intersection.x);
	intersection.y = std::round(intersection.y);
	if (intersection.x >= 0 || intersection.y >= 0)
		return true;

	return false;
}

void FilterBadComponents(std::vector<std::set<int> >& components) {
	int min_number_of_members = 6;

	components.erase(
			std::remove_if(components.begin(), components.end(),
					[min_number_of_members](std::set<int> component) {
						return component.size() < min_number_of_members;}), components.end());

}

double EuclideanDistance(cv::Point2f& p1, cv::Point2f& p2) {
	return cv::norm(p1 - p2);
}
double MIN_CORNER_DIST = 0.02; // of smallest image axis
double MIN_PARALLEL_LINE_DIST = 50.0;
double MIN_ACCEPTED_SCORE = 0.0;
double MIN_ADJACENT_LINE_ANGLE = 15.0;
double MAX_PARALLEL_LINE_ANGLE = 30.0;
double MAX_ANGLE_DIFF = 75.0;
double MAX_LINE_DIST = 0.05;
double MIN_PACKAGE_CONTOUR_LENGTH = 200.0;

} /* namespace internal */

} /* namespace automatic_package_measuring */
