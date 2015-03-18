#ifndef TEST_SRC_APMTESTCASE_H_
#define TEST_SRC_APMTESTCASE_H_

#include <opencv2/opencv.hpp>
#include <vector>

namespace automatic_package_measuring {

class APMTestCase {
public:
	APMTestCase(cv::Mat image, std::vector<cv::Point2i> reference_object, std::vector<cv::Point2i> package,
			cv::Vec3d dimensions);
	virtual ~APMTestCase();
	const std::vector<cv::Point2i>& getReferenceObject() const;
	const std::vector<cv::Point2i>& getPackage() const;
	const cv::Vec3d getDimensions() const;
	cv::Mat getImage() const;

private:
	cv::Mat image;
	const std::vector<cv::Point2i> reference_object;
	const std::vector<cv::Point2i> package;
	const cv::Vec3d dimensions; // [w, h, d]
};

} /* namespace automatic_package_measuring */

#endif /* TEST_SRC_APMTESTCASE_H_ */
