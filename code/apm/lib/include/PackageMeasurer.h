#ifndef INCLUDE_OPENCV2_PACKAGEMEASURER_H_
#define INCLUDE_OPENCV2_PACKAGEMEASURER_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

class PackageMeasurer {
public:
	PackageMeasurer();
	virtual ~PackageMeasurer();

	void measurePackage(cv::Mat image);
	std::vector<cv::Point>& getPaperCoordinates();
	void findReferenceObject();
private:

};

} /* namespace automatic_package_measuring */

#endif /* INCLUDE_OPENCV2_PACKAGEMEASURER_H_ */
