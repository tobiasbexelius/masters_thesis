#include "../include/ContourExtractor.h"

namespace automatic_package_measuring {

ContourExtractor::ContourExtractor(bool external_only): external_only(external_only) {
}

ContourExtractor::~ContourExtractor() {
}

std::vector<std::vector<cv::Point>> ContourExtractor::extractContours(cv::Mat edges) {
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	int mode;
	if(external_only)
		mode = CV_RETR_EXTERNAL;
	else
		mode = CV_RETR_TREE;
	cv::findContours(edges, contours, hierarchy, mode,
				CV_CHAIN_APPROX_SIMPLE);
	return contours;
}

} /* namespace automatic_package_measuring */
