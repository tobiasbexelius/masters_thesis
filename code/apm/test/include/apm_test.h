#ifndef TEST_SRC_APMTEST_H_
#define TEST_SRC_APMTEST_H_

#include <opencv2/opencv.hpp>

#include "../../lib/include/package_measurer.h"
#include "apm_test_case.h"

namespace automatic_package_measuring {

class APMTest {
public:
	APMTest(APMTestCase testCase, double max_error = 0.15,
			PackageMeasurer measurer = PackageMeasurer(), int rotation = 0);
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

	void MeasureWithKey();
	void MeasureWithKeyCalib();
	void MeasureCalib();

	bool isKeyMeasurementCorrect() const;
	bool isCalibKeyMeasurementCorrect() const;
	bool isCalibMeasurementCorrect() const;

	double getKeyMeasurementError() const;
	double getCalibKeyMeasurementError() const;
	double getCalibMeasurementError() const;


private:
	double nearestNeighbourError(std::vector<cv::Point2f> expected, std::vector<cv::Point2f> actual) const;
	double getCircumference(std::vector<cv::Point2f> points) const;
	bool testMeasurementCorrect(cv::Vec3f measurement) const;
	double calcMeasurementError(cv::Vec3f measurement) const;
	const APMTestCase test_case;
	const double max_error;
	std::vector<cv::Point2f> actual_reference_object;
	std::vector<cv::Point2f> actual_package;
	cv::Vec3f actual_measurement;
	PackageMeasurer measurer;
	int rotation;

	cv::Vec3f calib_measurement;
	cv::Vec3f key_measurement;
	cv::Vec3f calib_key_measurement;
};

} /* namespace automatic_package_measuring */

#endif /* TEST_SRC_APMTEST_H_ */
