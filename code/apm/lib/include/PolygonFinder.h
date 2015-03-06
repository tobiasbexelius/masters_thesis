#ifndef SRC_POLYGONFINDER_H_
#define SRC_POLYGONFINDER_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

class PolygonFinder {
public:
	PolygonFinder(double error_tolerance = 0.02, bool convex_only = true);
	virtual ~PolygonFinder();
	std::vector<std::vector<cv::Point2i>> findPolygons(std::vector<std::vector<cv::Point2i>> contours);
private:
	bool isConvex(std::vector<cv::Point2i> &polygon, double curve_length);
	const double error_tolerance;
	const double convex_only;
	const double convexity_defect_tolerance;
};

} /* namespace automatic_package_measuring */

#endif /* SRC_POLYGONFINDER_H_ */
