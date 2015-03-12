#include "../include/HoughLineTransform.h"

namespace automatic_package_measuring {

const int HoughLineTransform::DEFAULT_RHO = 2;
const double HoughLineTransform::DEFAULT_THETA = CV_PI / 180;
const int HoughLineTransform::DEFAULT_THRESHOLD = 50;
const int HoughLineTransform::DEFAULT_MIN_LENGTH = 80;
const int HoughLineTransform::DEFAULT_MAX_GAP = 55;

HoughLineTransform::HoughLineTransform(int rho, double theta, int threshold, int min_length, int max_gap) :
		rho(rho), theta(theta), threshold(threshold), min_length(min_length), max_gap(max_gap) {
}

HoughLineTransform::~HoughLineTransform() {
}

std::vector<cv::Vec4i> HoughLineTransform::detectLines(cv::Mat image) {
	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(image, lines, rho, theta, threshold, min_length, max_gap);
	return lines;
}

std::vector<cv::Vec2f> HoughLineTransform::detectLinesNonP(cv::Mat image) {
	std::vector<cv::Vec2f> lines;
	cv::HoughLines(image, lines, rho, theta, threshold, 0, 0);
	return lines;
}

} /* namespace automatic_package_measuring */
