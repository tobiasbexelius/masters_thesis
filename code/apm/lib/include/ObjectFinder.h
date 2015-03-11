#ifndef SRC_REFERENCEOBJECTFINDER_H_
#define SRC_REFERENCEOBJECTFINDER_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

typedef std::vector<cv::Point> Polygon;

class ObjectFinder {
public:
	ObjectFinder() {}
	virtual ~ObjectFinder(){}
	virtual std::vector<cv::Point> findObject(cv::Mat image, std::vector<std::vector<cv::Point>> contours) = 0;
};

} /* namespace automatic_package_measuring */

#endif /* SRC_REFERENCEOBJECTFINDER_H_ */
