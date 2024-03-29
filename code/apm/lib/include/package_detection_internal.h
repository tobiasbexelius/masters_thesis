#ifndef LIB_INCLUDE_PACKAGE_DETECTION_INTERNAL_H_
#define LIB_INCLUDE_PACKAGE_DETECTION_INTERNAL_H_

#include <unordered_map>
#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

namespace internal {

std::vector<int> GetLinesInPairs(const std::vector<std::tuple<int, int>>& line_pairs, std::vector<int> indices);
std::vector<cv::Point2f> FindCorners(const std::vector<cv::Vec4i>& lines,
		const std::vector<int>& line_indices, const cv::Size& image_size, const double min_corner_dist);
double RatePackage(std::vector<cv::Vec4i>& lines, std::vector<cv::Point2f>& package);
std::vector<std::vector<cv::Point2f>> FindPackages(const std::vector<cv::Vec4i>& lines,
		const std::vector<cv::Point2f>& reference_object, const cv::Size& image_size);
void FindParallelLines(const std::vector<cv::Vec4i>& lines, const double min_line_dist,
		std::vector<std::tuple<int, int>>& parallel_line_pairs, const double max_parallel_angle);
bool LineSegmentAngleComparator(const cv::Vec4i& a, const cv::Vec4i& b);
double LineSegmentAngle(const cv::Vec4i& line1, const cv::Vec4i& line2);
double LineSegmentDistance(const cv::Vec4i& line1, const cv::Vec4i& line2);
bool TryToCreatePolygon(const std::vector<cv::Vec4i>& lines, const std::vector<int> indices,
		const cv::Size& image_size, std::vector<cv::Point2f>& package);
bool EnclosesContour(const std::vector<cv::Point2f>& enclosing_contour, const std::vector<cv::Point2f>& enclosed_contour);

inline double EuclideanDistance(cv::Point2f& p1, cv::Point2f& p2);

extern double MIN_PARALLEL_LINE_DIST;
extern double MIN_ACCEPTED_SCORE;
extern double MIN_PACKAGE_CONTOUR_LENGTH;
extern double MAX_PARALLEL_LINE_ANGLE;
extern double MIN_CORNER_DIST;
extern int MAX_LINE_PAIRS;
extern double MIN_ACCEPTED_SUBSCORE;

} /* namespace automatic_package_measuring::internal */

} /* namespace automatic_package_measuring */

#endif /* LIB_INCLUDE_PACKAGE_DETECTION_INTERNAL_H_ */
