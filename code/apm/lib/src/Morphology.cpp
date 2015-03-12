#include "../include/Morphology.h"

namespace automatic_package_measuring {

Morphology::Morphology(int element_radius, int iterations) :
		element_radius(element_radius), iterations(iterations) {
}

Morphology::~Morphology() {
}

cv::Mat Morphology::close(cv::Mat image) {
	return morphology(image, cv::MORPH_CLOSE);
}

cv::Mat Morphology::open(cv::Mat image) {
	return morphology(image, cv::MORPH_OPEN);
}

cv::Mat Morphology::gradient(cv::Mat image) {
	return morphology(image, cv::MORPH_GRADIENT);
}

cv::Mat Morphology::morphology(cv::Mat image, int operation) {
	cv::Mat result;
	cv::Size ele_size = cv::Size(2 * element_radius + 1, 2 * element_radius + 1);
	cv::Point anchor = cv::Point(element_radius, element_radius);
	cv::Mat structuring_element = cv::getStructuringElement(cv::MORPH_RECT, ele_size, anchor);
	cv::morphologyEx(image, result, operation, structuring_element, anchor, iterations);

	return result;
}

} /* namespace automatic_package_measuring */
