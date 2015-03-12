#include "../include/APMTestCase.h"

namespace automatic_package_measuring {

APMTestCase::APMTestCase(cv::Mat image, std::vector<cv::Point2i> reference_object,
		std::vector<cv::Point2i> package, cv::Vec3d dimensions) :
		image(image), reference_object(reference_object), package(package), dimensions(dimensions) {
}

APMTestCase::~APMTestCase() {
}

const std::vector<cv::Point2i>& APMTestCase::getReferenceObject() const {
	return reference_object;
}

const std::vector<cv::Point2i>& APMTestCase::getPackage() const {
	return package;
}

cv::Mat APMTestCase::getImage() const {
	return image;
}

const cv::Vec3d APMTestCase::getDimensions() const {
	return dimensions;
}

} /* namespace automatic_package_measuring */
