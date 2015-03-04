#ifndef TEST_SRC_APMTESTCASE_H_
#define TEST_SRC_APMTESTCASE_H_

#include <opencv2/opencv.hpp>
#include <vector>

namespace automatic_package_measuring {

class APMTestCase {
public:
	APMTestCase(cv::Mat image, std::vector<cv::Point2i> corners, double width,
			double height, double depth);
	virtual ~APMTestCase();
	const std::vector<cv::Point2i>& getCorners() const;
	double getDepth() const;
	double getHeight() const;
	const cv::Mat& getImage() const;
	double getWidth() const;

private:
	const cv::Mat image;
	const std::vector<cv::Point2i> corners;
	const double width, height, depth;
};

} /* namespace automatic_package_measuring */

#endif /* TEST_SRC_APMTESTCASE_H_ */
