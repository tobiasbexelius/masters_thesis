#ifndef TEST_SRC_APMTEST_H_
#define TEST_SRC_APMTEST_H_

#include <opencv2/opencv.hpp>

#include "../../lib/include/package_measurer.h"
#include "apm_test_case.h"

namespace automatic_package_measuring {

class APMTest {
public:
	APMTest(APMTestCase testCase, double max_error = 0.05, PackageMeasurer measurer = PackageMeasurer());
	virtual ~APMTest();
	void run();
	bool success() const;
	bool isReferenceObjectCorrect() const;
	double getReferenceObjectError() const;
	std::vector<cv::Point2i> getExpectedReferenceObject() const;
	std::vector<cv::Point2i> getActualReferenceObject() const;
	bool isPackageCorrect() const;
	double getPackageError() const;
	std::vector<cv::Point2i> getExpectedPackage() const;
	std::vector<cv::Point2i> getActualPackage() const;
	bool isMeasurementCorrect() const;
	double getMeasurementError() const;
	cv::Vec3d getExpectedMeasurement() const;
	cv::Vec3d getActualMeasurement() const;

private:
	double nearestNeighbourError(std::vector<cv::Point2i> expected, std::vector<cv::Point2i> actual) const;
	double getCircumference(std::vector<cv::Point2i> points) const;
	const APMTestCase test_case;
	const double max_error;
	std::vector<cv::Point2i> actual_reference_object;
	std::vector<cv::Point2i> actual_package;
	cv::Vec3d actual_measurement;
	PackageMeasurer measurer;
};

} /* namespace automatic_package_measuring */

#endif /* TEST_SRC_APMTEST_H_ */
