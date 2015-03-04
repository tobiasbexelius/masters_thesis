#include "../include/APMTest.h"

namespace automatic_package_measuring {

APMTest::APMTest(APMTestCase testCase, double max_error) :
		testCase(testCase), max_error(max_error) {
}

APMTest::~APMTest() {
}

void APMTest::run() {
}

bool APMTest::success() const {
	return true;
}

std::vector<cv::Point2i> APMTest::getExpectedCorners() const {
	return testCase.getCorners();
}

double APMTest::getError() {
	return -1.0;
}

std::vector<cv::Point2i> APMTest::getActualCorners() const {
	return actualCorners;
}

} /* namespace automatic_package_measuring */
