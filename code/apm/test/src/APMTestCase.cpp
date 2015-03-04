/*
 * APMTestCase.cpp
 *
 *  Created on: Mar 4, 2015
 *      Author: tobias
 */

#include "../include/APMTestCase.h"

namespace automatic_package_measuring {

APMTestCase::APMTestCase(cv::Mat image, std::vector<cv::Point2i> corners,
		double width, double height, double depth) :
		image(image), corners(corners), width(width), height(height), depth(
				depth) {
}

APMTestCase::~APMTestCase() {
}

const std::vector<cv::Point2i>& APMTestCase::getCorners() const {
	return corners;
}

double APMTestCase::getDepth() const {
	return depth;
}

double APMTestCase::getHeight() const {
	return height;
}

const cv::Mat& APMTestCase::getImage() const {
	return image;
}

double APMTestCase::getWidth() const {
	return width;
}

} /* namespace automatic_package_measuring */
