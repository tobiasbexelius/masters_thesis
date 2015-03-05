#ifndef SRC_CONTOUREXTRACTOR_H_
#define SRC_CONTOUREXTRACTOR_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

class ContourExtractor {
public:
	ContourExtractor(bool external_only = false);
	virtual ~ContourExtractor();
	std::vector<std::vector<cv::Point2i>> extractContours(cv::Mat edges);
private:
	bool external_only;

};

} /* namespace automatic_package_measuring */

#endif /* SRC_CONTOUREXTRACTOR_H_ */
