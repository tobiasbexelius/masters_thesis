#ifndef LIB_INCLUDE_PACKAGE_DETECTION_INTERNAL_H_
#define LIB_INCLUDE_PACKAGE_DETECTION_INTERNAL_H_

#include <unordered_map>
#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

namespace internal {

struct Intersection {
	int l1;
	int l2;
};

struct Package {
	std::vector<cv::Point2f> corners;
	std::unordered_map<int, int> line_to_pair;
	std::unordered_map<int, std::vector<int>> pair_to_lines;
};

double RatePackage(std::vector<cv::Vec4i>& lines, Package& package);
std::vector<Package> FindPackages(std::vector<cv::Vec4i>& lines, std::vector<cv::Point2f>& reference_object);
void FindParallelLines(std::vector<cv::Vec4i>& lines, std::vector<std::tuple<int, int>>& parallel_line_pairs);
bool LineSegmentAngleComparator(const cv::Vec4i& a, const cv::Vec4i& b);
double LineSegmentAngle(const cv::Vec4i& line1, const cv::Vec4i& line2);
bool FindIntersection(const cv::Vec4i& line1, const cv::Vec4i& line2, cv::Point2f& intersection);
void FindConnectedComponents(const std::vector<std::vector<Intersection>>& intersections,
		std::vector<std::set<int>>& connected_components);
void FilterBadComponents(std::vector<std::set<int>>& components);
bool HasDuplicates(std::vector<int>& vec);
void GetPoints(std::vector<cv::Vec4i>& lines, std::vector<int>& points, std::vector<cv::Point2f>& points_out);
double LineSegmentDistance(const cv::Vec4i& line1, const cv::Vec4i& line2);

void FindCorners(const std::vector<cv::Vec4i>& lines,
		const std::unordered_map<int, std::vector<int>>& neighbour_list, std::vector<cv::Point2f>& corners);
std::unordered_map<int, std::vector<int>> FindNeighbouringLines(const std::vector<cv::Vec4i>& lines,
		std::unordered_map<int, int> line_to_pair);

bool TryToCreatePackage(std::vector<cv::Vec4i> lines, std::vector<std::tuple<int, int>> line_pairs,
		std::vector<int> indices, Package& package);
bool EnclosesContour(std::vector<cv::Point>& enclosing_contour, std::vector<cv::Point>& enclosed_contour);

extern double MIN_PARALLEL_LINE_DIST;
extern double MIN_ACCEPTED_SCORE;
extern double MIN_ADJACENT_LINE_ANGLE;
extern double MAX_ANGLE_DIFF;
extern double MAX_LINE_DIST;
extern double MIN_PACKAGE_CONTOUR_LENGTH;
extern double MAX_PARALLEL_LINE_ANGLE;

} /* namespace automatic_package_measuring::internal */

} /* namespace automatic_package_measuring */

#endif /* LIB_INCLUDE_PACKAGE_DETECTION_INTERNAL_H_ */
