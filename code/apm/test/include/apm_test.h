#ifndef TEST_SRC_APMTEST_H_
#define TEST_SRC_APMTEST_H_

#include <opencv2/opencv.hpp>

#include "../../lib/include/package_measurer.h"
#include "apm_test_case.h"

namespace automatic_package_measuring {

class APMTest {
public:
	APMTest(APMTestCase testCase, double max_error = 0.1, PackageMeasurer measurer = PackageMeasurer(), cv::Vec2f reference_object_size = cv::Vec2f(297,210), int rotation = 0);
	virtual ~APMTest();
	void run();
	bool success() const;
	bool isReferenceObjectCorrect() const;
	double getReferenceObjectError() const;
	std::vector<cv::Point2f> getExpectedReferenceObject() const;
	std::vector<cv::Point2f> getActualReferenceObject() const;
	bool isPackageCorrect() const;
	double getPackageError() const;
	std::vector<cv::Point2f> getExpectedPackage() const;
	std::vector<cv::Point2f> getActualPackage() const;
	bool isMeasurementCorrect() const;
	double getMeasurementError() const;
	cv::Vec3f getExpectedMeasurement() const;
	cv::Vec3f getActualMeasurement() const;

private:
	double nearestNeighbourError(std::vector<cv::Point2f> expected, std::vector<cv::Point2f> actual) const;
	double getCircumference(std::vector<cv::Point2f> points) const;
	const APMTestCase test_case;
	const double max_error;
	std::vector<cv::Point2f> actual_reference_object;
	std::vector<cv::Point2f> actual_package;
	cv::Vec3f actual_measurement;
	PackageMeasurer measurer;
	cv::Vec2f reference_object_size;
	int rotation;
};

} /* namespace automatic_package_measuring */

#endif /* TEST_SRC_APMTEST_H_ */
