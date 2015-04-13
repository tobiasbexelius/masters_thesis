#ifndef TEST_SRC_APMTESTCASE_H_
#define TEST_SRC_APMTESTCASE_H_

#include "json/json.h"
#include <opencv2/opencv.hpp>
#include <vector>

namespace automatic_package_measuring {

class APMTestCase {
public:
	APMTestCase();
	APMTestCase(std::string file_name, int package_id, std::string reference_object_type,
			cv::Vec3f dimensions, double distance);
	APMTestCase(cv::Mat image, std::vector<cv::Point2f> reference_object, std::vector<cv::Point2f> package, cv::Vec3f dimensions);
	virtual ~APMTestCase();
	const std::vector<cv::Point2f>& GetReferenceObject() const;
	const std::vector<cv::Point2f>& GetPackage() const;
	const cv::Vec3f GetDimensions() const;
	cv::Mat GetImage() const;

	void AppendReferenceObject(int x, int y);
	void AppnedPackage(int x, int y);
	Json::Value AsJson();
private:
	double distance;
	int package_id;
	std::string file_name;
	std::string reference_object_type;
	cv::Mat image;
	std::vector<cv::Point2f> reference_object;
	std::vector<cv::Point2f> package;
	cv::Vec3f dimensions; // [w, h, d]
};

} /* namespace automatic_package_measuring */

#endif /* TEST_SRC_APMTESTCASE_H_ */
