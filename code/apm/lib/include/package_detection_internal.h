#ifndef LIB_INCLUDE_PACKAGE_DETECTION_INTERNAL_H_
#define LIB_INCLUDE_PACKAGE_DETECTION_INTERNAL_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

namespace internal {

bool LineIntersection(cv::Point2f l1_start, cv::Point2f l1_end, cv::Point2f l2_start, cv::Point2f l2_end,
		cv::Point2f &intersection);
void FindCorners(const std::vector<cv::Vec4i>& lines, std::vector<cv::Point>& corners,
		double max_angle_diff_in_degrees, double max_line_dist);

extern double MAX_ANGLE_DIFF;
extern double MAX_DIST;
extern double MIN_PACKAGE_CONTOUR_LENGTH;

} /* namespace automatic_package_measuring::internal */

} /* namespace automatic_package_measuring */

#endif /* LIB_INCLUDE_PACKAGE_DETECTION_INTERNAL_H_ */
