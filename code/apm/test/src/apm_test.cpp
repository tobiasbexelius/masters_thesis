#include <limits>
#include <cmath>
#include "../include/apm_test.h"
#include "../../lib/include/package_measuring.h"

namespace automatic_package_measuring {

APMTest::APMTest(APMTestCase test_case, double max_error, PackageMeasurer measurer, int rotation) :
		test_case(test_case),
		max_error(max_error),
		measurer(measurer),
		rotation(rotation) {
}

APMTest::~APMTest() {
}

void APMTest::run() {
	measurer.SetReferenceObjectSize(test_case.GetReferenceObjectSize());
	cv::Mat image = test_case.GetImage();
	measurer.AnalyzeImage(image, rotation);
	actual_reference_object = measurer.GetReferenceObject();
	actual_package = measurer.GetPackage();
	actual_measurement = measurer.GetMeasurements();

}

bool APMTest::success() const {
	return isReferenceObjectCorrect() && isPackageCorrect() && isMeasurementCorrect();
}

std::vector<cv::Point2f> APMTest::getExpectedPackage() const {
	return test_case.GetPackage();
}

double APMTest::getPackageError() const {
	if (test_case.GetPackage().empty())
		return 1;

	double err = nearestNeighbourError(test_case.GetPackage(), actual_package)
			/ getCircumference(test_case.GetPackage());
	return std::fmin(err, 1.0);
}

double APMTest::getReferenceObjectError() const {
	if (test_case.GetReferenceObject().empty())
		return 1;

	double err = nearestNeighbourError(test_case.GetReferenceObject(), actual_reference_object)
			/ getCircumference(test_case.GetReferenceObject());
	return std::fmin(err, 1.0);
}

std::vector<cv::Point2f> APMTest::getExpectedReferenceObject() const {
	return test_case.GetReferenceObject();
}

std::vector<cv::Point2f> APMTest::getActualReferenceObject() const {
	return actual_reference_object;
}

bool APMTest::isReferenceObjectCorrect() const {
	return getReferenceObjectError() < max_error;
}

bool APMTest::isPackageCorrect() const {
	return getPackageError() < max_error;
}

std::vector<cv::Point2f> APMTest::getActualPackage() const {
	return actual_package;
}

bool APMTest::isMeasurementCorrect() const {
	return testMeasurementCorrect(actual_measurement);
}

bool APMTest::testMeasurementCorrect(cv::Vec3f measurement) const {
	if(measurement[0] ==0||measurement[1] == 0 || measurement[2]==0)
				return false;

		cv::Vec3f expected = test_case.GetDimensions();
		std::vector<double> a_vec = { measurement[0], measurement[1], measurement[2] };
		std::vector<double> e_vec = { expected[0], expected[1], expected[2] };
		std::sort(a_vec.begin(), a_vec.end());
		std::sort(e_vec.begin(), e_vec.end());

		double err1 = std::abs(1 - a_vec[0] / e_vec[0]);
		double err2 = std::abs(1 - a_vec[1] / e_vec[1]);
		double err3 = std::abs(1 - a_vec[2] / e_vec[2]);

		return err1 < max_error && err2 < max_error && err3 < max_error;
}

double APMTest:: calcMeasurementError(cv::Vec3f measurement) const {
	if(measurement[0] ==0||measurement[1] == 0 || measurement[2]==0)
				return 1;

		cv::Vec3f expected = test_case.GetDimensions();
		std::vector<double> a_vec = { measurement[0], measurement[1], measurement[2] };
		std::vector<double> e_vec = { expected[0], expected[1], expected[2] };
		std::sort(a_vec.begin(), a_vec.end());
		std::sort(e_vec.begin(), e_vec.end());

		double err1 = std::abs(1 - a_vec[0] / e_vec[0]);
		double err2 = std::abs(1 - a_vec[1] / e_vec[1]);
		double err3 = std::abs(1 - a_vec[2] / e_vec[2]);

		return err1 + err2 + err3;

}

double APMTest::getMeasurementError() const {
return calcMeasurementError(actual_measurement);
}

cv::Vec3f APMTest::getExpectedMeasurement() const {
	return test_case.GetDimensions();
}

cv::Vec3f APMTest::getActualMeasurement() const {
	return actual_measurement;
}

/*
 * Returns the accumlated error for all points
 * */
double APMTest::nearestNeighbourError(std::vector<cv::Point2f> expected,
		std::vector<cv::Point2f> actual) const {
	if (expected.size() != actual.size())
		return std::numeric_limits<int>::max();

	cv::Mat_<float> flannPoints(expected.size(), 2);
	for (int i = 0; i < expected.size(); ++i) {
		flannPoints[i][0] = (float) expected[i].x;
		flannPoints[i][1] = (float) expected[i].y;
	}

	cv::flann::Index flann_index(flannPoints, cv::flann::LinearIndexParams());
	std::vector<int> indices = std::vector<int>(1);
	std::vector<float> distances = std::vector<float>(1);
	std::vector<float> query = std::vector<float>(2);
	std::set<int> used_indices;

	double error = 0;

	for (int i = 0; i < actual.size(); ++i) {

		query[0] = (float) actual[i].x;
		query[1] = (float) actual[i].y;

		flann_index.knnSearch(query, indices, distances, 1, cv::flann::SearchParams());

		if (used_indices.count(indices[0]) > 0) { // same corner cannot be used twice
			//	std::cerr << "Same point used more than once in nearest neighbour." << std::endl;
			return std::numeric_limits<int>::max();;
		}
		used_indices.insert(indices[0]);
		error += std::sqrt(distances[0]);
	}

	return error;
}

bool APMTest::isKeyMeasurementCorrect() const {
	return testMeasurementCorrect(key_measurement);
}

bool APMTest::isCalibKeyMeasurementCorrect() const {
	return testMeasurementCorrect(calib_key_measurement);
}

bool APMTest::isCalibMeasurementCorrect() const {
	return testMeasurementCorrect(calib_measurement);
}

double APMTest::getKeyMeasurementError() const {
return calcMeasurementError(key_measurement);
}

double APMTest::getCalibKeyMeasurementError() const {
	return calcMeasurementError(calib_key_measurement);
}

double APMTest::getCalibMeasurementError() const {
	return calcMeasurementError(calib_measurement);
}

/**
 * Assumes that subsequent points are connected and that the first and last points are connected
 */
double APMTest::getCircumference(std::vector<cv::Point2f> points) const {
	double circumference = 0;

	auto it = points.begin();
	cv::Point2i first = *it;
	cv::Point2i previous = first;
	for (++it; it != points.end(); ++it) {
		cv::Point2i current = *it;
		circumference += cv::norm(current - previous);
		previous = current;
	}
	circumference += cv::norm(first - previous);
	return circumference;
}

void APMTest::MeasureWithKey() {
	std::vector<cv::Point2f> reference_object = test_case.GetReferenceObject();
	std::vector<cv::Point2f> package = test_case.GetPackage();
	cv::Vec3i edges;
	cv::Vec3f measurement = MeasurePackage(test_case.GetImage().size(), reference_object, test_case.GetReferenceObjectSize(), package, edges, true);

}

void APMTest::MeasureWithKeyCalib() {
	std::vector<cv::Point2f> reference_object = test_case.GetReferenceObject();
	std::vector<cv::Point2f> package = test_case.GetPackage();
	cv::Vec3i edges;
	cv::Vec3f measurement = MeasurePackage(test_case.GetImage().size(), reference_object, test_case.GetReferenceObjectSize(), package, edges, false);
}

void APMTest::MeasureCalib(){
	cv::Vec3i edges;
	MeasurePackage(test_case.GetImage().size(), actual_reference_object, test_case.GetReferenceObjectSize(), actual_package, edges, false);
}


} /* namespace automatic_package_measuring */
