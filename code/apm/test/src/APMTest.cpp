#include "../include/APMTest.h"

#include <limits>
#include <cmath>

namespace automatic_package_measuring {

APMTest::APMTest(APMTestCase testCase, double max_error,
		PackageMeasurer measurer) :
		test_case(testCase), max_error(max_error), measurer(measurer) {
}

APMTest::~APMTest() {
}

void APMTest::run() {
	PackageMeasurer measurer;
	measurer.analyzeImage(test_case.getImage());
	actual_reference_object = measurer.getReferenceObject();
	actual_package = measurer.getPackage();
	actual_measurement = measurer.getMeasurements();
}

bool APMTest::success() const {
	return isReferenceObjectCorrect() && isPackageCorrect()
			&& isMeasurementCorrect();
}

std::vector<cv::Point2i> APMTest::getExpectedPackage() const {
	return test_case.getPackage();
}

double APMTest::getPackageError() const {
	double err = nearestNeighbourError(test_case.getPackage(), actual_package)
			/ getCircumference(test_case.getPackage());
	return std::fmin(err, 1.0);
}

double APMTest::getReferenceObjectError() const {
	double err = nearestNeighbourError(test_case.getReferenceObject(),
			actual_reference_object)
			/ getCircumference(test_case.getReferenceObject());
	return std::fmin(err, 1.0);
}

std::vector<cv::Point2i> APMTest::getExpectedReferenceObject() const {
	return test_case.getReferenceObject();
}

std::vector<cv::Point2i> APMTest::getActualReferenceObject() const {
	return actual_reference_object;
}

bool APMTest::isReferenceObjectCorrect() const {
	return getReferenceObjectError() < max_error;
}

bool APMTest::isPackageCorrect() const {
	return getPackageError() < max_error;
}

std::vector<cv::Point2i> APMTest::getActualPackage() const {
	return actual_package;
}

bool APMTest::isMeasurementCorrect() const {
	return getMeasurementError() < max_error;
}

double APMTest::getMeasurementError() const {
	return cv::norm(test_case.getDimensions() - actual_measurement)
			/ cv::norm(test_case.getDimensions());
}

cv::Vec3d APMTest::getExpectedMeasurement() const {
	return test_case.getDimensions();
}

cv::Vec3d APMTest::getActualMeasurement() const {
	return actual_measurement;
}

/*
 * Returns the accumlated error for all points
 * */
double APMTest::nearestNeighbourError(std::vector<cv::Point2i> expected,
		std::vector<cv::Point2i> actual) const {

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

		flann_index.knnSearch(query, indices, distances, 1,
				cv::flann::SearchParams());

		if (used_indices.count(indices[0]) > 0) { // same corner cannot be used twice
			std::cerr << "Same point used more than once in nearest neighbour."
					<< std::endl;
			return std::numeric_limits<int>::max();;
		}
		used_indices.insert(indices[0]);
		error += std::sqrt(distances[0]);
	}

	return error;
}

/**
 * Assumes that subsequent points are connected and that the first and last points are connected
 */
double APMTest::getCircumference(std::vector<cv::Point2i> points) const {
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

} /* namespace automatic_package_measuring */
