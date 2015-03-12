#ifndef LIB_INCLUDE_PACKAGEFINDER_H_
#define LIB_INCLUDE_PACKAGEFINDER_H_

#include <opencv2/opencv.hpp>
#include "ObjectFinder.h"

namespace automatic_package_measuring {

class PackageFinder: ObjectFinder {
public:
	PackageFinder();
	virtual ~PackageFinder();
	std::vector<cv::Point> findObject(cv::Mat image, std::vector<std::vector<cv::Point>> polygons);
	bool lineIntersection(cv::Point2f l1_start, cv::Point2f l1_end, cv::Point2f l2_start, cv::Point2f l2_end,
			cv::Point2f &intersection);
	void findCorners(const std::vector<cv::Vec4i>& lines, std::vector<cv::Point2i>& corners,
			double max_angle_diff_in_degrees, double max_line_dist);
private:
	double MAX_ANGLE_DIFF = 75.0;
	double MAX_DIST = 0.1;
};

} /* namespace automatic_package_measuring */

#endif /* LIB_INCLUDE_PACKAGEFINDER_H_ */
