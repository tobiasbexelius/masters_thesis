#ifndef LIB_INCLUDE_PACKAGE_DETECTION_INTERNAL_H_
#define LIB_INCLUDE_PACKAGE_DETECTION_INTERNAL_H_

#include <unordered_map>
#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

namespace internal {

struct Package {
	std::vector<cv::Point2f> corners;
	std::unordered_map<int, int> line_to_pair;
	//std::unordered_map<int, std::vector<int>> pair_to_lines;
};

std::vector<cv::Point2f> FindCorners(const std::vector<cv::Vec4i>& lines,
		const std::vector<int>& line_indices, const double min_corner_dist);
double RatePackage(std::vector<cv::Vec4i>& lines, Package& package);
std::vector<Package> FindPackages(const std::vector<cv::Vec4i>& lines,
		const std::vector<cv::Point2f>& reference_object, const cv::Size& image_size);
void FindParallelLines(const std::vector<cv::Vec4i>& lines, const double min_line_dist,
		std::vector<std::tuple<int, int>>& parallel_line_pairs);
bool LineSegmentAngleComparator(const cv::Vec4i& a, const cv::Vec4i& b);
double LineSegmentAngle(const cv::Vec4i& line1, const cv::Vec4i& line2);
bool FindIntersection(const cv::Vec4i& line1, const cv::Vec4i& line2, cv::Point2f& intersection);
double LineSegmentDistance(const cv::Vec4i& line1, const cv::Vec4i& line2);
bool TryToCreatePackage(const std::vector<cv::Vec4i>& lines,
		const std::vector<std::tuple<int, int>>& line_pairs, const std::vector<int>& line_pair_indices,
		const cv::Size& image_size, Package& package);
bool EnclosesContour(std::vector<cv::Point>& enclosing_contour, std::vector<cv::Point>& enclosed_contour);
inline double EuclideanDistance(cv::Point2f& p1, cv::Point2f& p2);

extern double MIN_PARALLEL_LINE_DIST;
extern double MIN_ACCEPTED_SCORE;
//extern double MIN_ADJACENT_LINE_ANGLE;
extern double MIN_PACKAGE_CONTOUR_LENGTH;
extern double MAX_PARALLEL_LINE_ANGLE;
extern double MIN_CORNER_DIST;


} /* namespace automatic_package_measuring::internal */

} /* namespace automatic_package_measuring */

#endif /* LIB_INCLUDE_PACKAGE_DETECTION_INTERNAL_H_ */
