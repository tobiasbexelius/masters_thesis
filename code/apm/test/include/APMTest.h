#ifndef TEST_SRC_APMTEST_H_
#define TEST_SRC_APMTEST_H_

namespace automatic_package_measuring {

class APMTest {
public:
	APMTest(APMTestCase testCase, double max_error = 0.05);
	virtual ~APMTest();
	void run();
	double getError();
	bool success() const;
	std::vector<cv::Point2i> getExpectedCorners() const;
	std::vector<cv::Point2i> getActualCorners() const;
private:
	const APMTestCase testCase;
	const double max_error;
	std::vector<cv::Point2i> actualCorners;
};

} /* namespace automatic_package_measuring */

#endif /* TEST_SRC_APMTEST_H_ */
