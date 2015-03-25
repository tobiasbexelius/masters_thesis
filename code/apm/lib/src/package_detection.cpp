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
//cv::Mat derp;

std::vector<cv::Point> FindPackage(const cv::Mat& image, const cv::Mat& edges,
		std::vector<cv::Point>& reference_object) {
	cv::Mat edges_cpy = edges.clone();
	std::vector<std::vector<cv::Point>> contours;
	FindContours(edges_cpy, true, contours);
	PruneShortContours(contours, MIN_PACKAGE_CONTOUR_LENGTH);
	PrunePeripheralContours(contours, image.size());
//	derp = image.clone();
	if (contours.empty())
		return std::vector<cv::Point>();

	cv::Mat contours_mat = cv::Mat(image.size(), CV_8UC1, cv::Scalar(0));
	cv::drawContours(contours_mat, contours, -1, cv::Scalar(255));

	std::vector<cv::Vec4i> lines;
	DetectLines(contours_mat, lines);

	//std::cout << "lines " << lines.size() << std::endl;

	std::vector<Package> packages = FindPackages(lines, reference_object);
	int max_score = std::numeric_limits<int>::min();
	Package max_package;

//	int packages_size = packages.size();
//	std::cout << "size " << packages.size() << std::endl;
	for (auto it = packages.begin(); it != packages.end(); ++it) {
//		if (packages_size > 50)
//			break;
//		derp = image.clone();
//
//		for (int i = 0; i < it->corners.size(); ++i) {
//			cv::circle(derp, it->corners[i], 2, cv::Scalar(0, 0, 255), 3);
//		}

		double score = RatePackage(lines, *it);
		if (score > max_score) {
			max_score = score;
			max_package = *it;
		}

//		std::cout << "current score: " << score << std::endl;
//
//		imshow("derp", derp);
//		cv::waitKey(0);
	}

	if (max_score < MIN_ACCEPTED_SCORE)
		return std::vector<cv::Point>();

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

void DrawLine(std::vector<cv::Vec4i>& lines, cv::Mat& canvas, int index, int intensity) {
	cv::line(canvas, cv::Point(lines[index][0], lines[index][1]), cv::Point(lines[index][2], lines[index][3]),
			cv::Scalar(255, 0, intensity), 3);
}

std::vector<Package> FindPackages(std::vector<cv::Vec4i>& lines, std::vector<cv::Point>& reference_object) {
	std::vector<Package> packages;
	std::vector<std::tuple<int, int>> line_pairs;
	FindParallelLines(lines, line_pairs);
	if (line_pairs.size() < 3)
		return packages;

//	for (int i = 0; i < lines.size(); ++i) {
//		std::cout << "Line: " << lines[i] << "( " << i << ")" << std::endl;
//	}
//	std::cout << "Num parallel pairs: " << line_pairs.size() << std::endl;
//	for (int i = 0; i < line_pairs.size(); ++i) {
//		DrawLine(lines, derp, std::get<0>(line_pairs[i]), i * 30);
//		DrawLine(lines, derp, std::get<1>(line_pairs[i]), i * 30);
//		cv::imshow("derp", derp);
//		cv::waitKey(0);
//	}
	if (line_pairs.size() > 50) {
		std::cout << "Too many line pairs: " << line_pairs.size() << std::endl;
		return std::vector<Package>();
	}

	for (int i = 0; i < line_pairs.size(); ++i) {
		for (int j = i + 1; j < line_pairs.size(); ++j) {
			for (int k = j + 1; k < line_pairs.size(); ++k) {

				std::vector<int> indices = { i, j, k };
				Package package;
				bool isPackageValid = TryToCreatePackage(lines, line_pairs, indices, package);
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

bool TryToCreatePackage(std::vector<cv::Vec4i> lines, std::vector<std::tuple<int, int>> line_pairs,
		std::vector<int> indices, Package& package) {

	for (int i = 0; i < indices.size(); ++i) {
		package.line_to_pair[std::get<0>(line_pairs[indices[i]])] = indices[i];
		package.line_to_pair[std::get<1>(line_pairs[indices[i]])] = indices[i];
	}

	if (package.line_to_pair.size() != 6) // contains doubles
		return false;

	for (auto it = package.line_to_pair.begin(); it != package.line_to_pair.end(); ++it) {
		if (!package.pair_to_lines.count(it->second))
			package.pair_to_lines[it->second] = std::vector<int>();

		package.pair_to_lines[it->second].push_back(it->first);
	}

	std::unordered_map<int, std::vector<int>> neighbour_list = FindNeighbouringLines(lines,
			package.line_to_pair);

	for (auto it = neighbour_list.begin(); it != neighbour_list.end(); ++it) {
		int line = it->first;
		int neighbour = it->second[0];
		if (LineSegmentAngle(lines[line], lines[neighbour]) < MIN_ADJACENT_LINE_ANGLE)
			return false;

		neighbour = it->second[1];
		if (LineSegmentAngle(lines[line], lines[neighbour]) < MIN_ADJACENT_LINE_ANGLE)
			return false;

	}
	std::vector<cv::Point> corners;
	FindCorners(lines, neighbour_list, corners);

	if (corners.size() != 6)
		return false;

	std::vector<int> hull;
	cv::convexHull(corners, hull);

	if (hull.size() != 6)
		return false;

	for (int i = 0; i < hull.size(); ++i) {
		package.corners.push_back(corners[hull[i]]);
	}

	return true;
}

void FindCorners(const std::vector<cv::Vec4i>& lines,
		const std::unordered_map<int, std::vector<int>>& neighbour_list, std::vector<cv::Point>& corners) { // TODO speed this up
	for (auto it = neighbour_list.begin(); it != neighbour_list.end(); ++it) {
		int line = it->first;
		std::vector<int> neighbours = it->second;
		cv::Point intersection;
		if (FindIntersection(lines[line], lines[neighbours[0]], intersection)
				&& std::find(corners.begin(), corners.end(), intersection) == corners.end())
			corners.push_back(intersection);
		if (FindIntersection(lines[line], lines[neighbours[1]], intersection)
				&& std::find(corners.begin(), corners.end(), intersection) == corners.end())
			corners.push_back(intersection);
	}
}

std::unordered_map<int, std::vector<int>> FindNeighbouringLines(const std::vector<cv::Vec4i>& lines,
		std::unordered_map<int, int> line_to_pair) {
	std::unordered_map<int, std::vector<int>> neighbour_list;

	auto tuple_comp =
			[]( std::tuple<double,int> a, std::tuple<double,int> b ) {return std::get<0>(a) > std::get<0>(b);};

	for (auto it = line_to_pair.begin(); it != line_to_pair.end(); ++it) {

		std::priority_queue<std::tuple<double, int>, std::vector<std::tuple<double, int>>,
				decltype( tuple_comp )> distances(tuple_comp);

		int line1 = it->first;
		for (auto it2 = line_to_pair.begin(); it2 != line_to_pair.end(); ++it2) {

			int line2 = it2->first;
			if (line_to_pair[line1] == line_to_pair[line2])
				continue;

			distances.push(std::tuple<double, int>(LineSegmentDistance(lines[line1], lines[line2]), line2));
		}

		neighbour_list[it->first] = std::vector<int>();
		neighbour_list[line1].push_back(std::get<1>(distances.top()));
		distances.pop();
		neighbour_list[line1].push_back(std::get<1>(distances.top()));

	}
	return neighbour_list;
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

void FindParallelLines(std::vector<cv::Vec4i>& lines,
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
			ds.Union(intersections[i][j].l1, intersections[i][j].l2);
		}
	}
	connected_components = ds.AsSets();

}

bool FindIntersection(const cv::Vec4i& line1, const cv::Vec4i& line2, cv::Point& intersection) {
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

double MIN_PARALLEL_LINE_DIST = 50.0;
double MIN_ACCEPTED_SCORE = 0.0;
double MIN_ADJACENT_LINE_ANGLE = 15.0;
double MAX_PARALLEL_LINE_ANGLE = 30.0;
double MAX_ANGLE_DIFF = 75.0;
double MAX_LINE_DIST = 0.05;
double MIN_PACKAGE_CONTOUR_LENGTH = 200.0;

} /* namespace internal */

} /* namespace automatic_package_measuring */
