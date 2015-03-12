#include "../include/ContourExtractor.h"
#include <assert.h>

namespace automatic_package_measuring {

ContourExtractor::ContourExtractor() {
}

ContourExtractor::~ContourExtractor() {
}

std::vector<std::vector<cv::Point>> ContourExtractor::extractContours(cv::Mat edges, bool external_only) {

	std::vector<std::vector<cv::Point2i>> contours;
	std::vector<cv::Vec4i> hierarchy;

	int mode;
	if (external_only)
		mode = CV_RETR_EXTERNAL;
	else
		mode = CV_RETR_LIST;

	cv::findContours(edges, contours, hierarchy, mode, CV_CHAIN_APPROX_SIMPLE);

	return contours;
}

/**
 * Removes contours shorter than min_length
 */
void ContourExtractor::pruneShortContours(std::vector<std::vector<cv::Point> >& contours, double min_length) {

	contours.erase(
			std::remove_if(contours.begin(), contours.end(), [min_length](std::vector<cv::Point>& contour) {
				return cv::arcLength(contour, false) < min_length;}), contours.end());
}

/**
 * Removes contours which center is outside placement_constraint.
 * placement_constraint is should be in range (0, 0.5) and decides
 * how close to the center of the image, that the center of a contour
 *  needs to be.
 *
 *  A value of 0.25 will prune contours which center is
 *  outside the ranges (0.25, 0.75) times the image dimensions on either axis.
 */
void ContourExtractor::prunePeripheralContours(std::vector<std::vector<cv::Point> >& contours,
		const cv::Mat& image, double placement_constraint) {

	int img_width = image.cols;
	int img_height = image.rows;
	contours.erase(
			std::remove_if(contours.begin(), contours.end(),
					[placement_constraint, img_height, img_width](std::vector<cv::Point>& contour) {
						return !isContourCentered(contour, placement_constraint, img_height, img_width);}),
			contours.end());
}

bool ContourExtractor::isContourCentered(std::vector<cv::Point>& contour, double placement_constraint,
		int img_height, int img_width) {

	assert(placement_constraint >= 0 && placement_constraint <= 1);

	cv::Rect bounding_rect = cv::boundingRect(contour);

	int center_x = bounding_rect.x + bounding_rect.width / 2;
	int center_y = bounding_rect.y + bounding_rect.height / 2;

	int min_x = img_width * placement_constraint;
	int min_y = img_height * placement_constraint;

	int max_x = img_width * (1 - placement_constraint);
	int max_y = img_height * (1 - placement_constraint);

	return center_x >= min_x && center_y >= min_y && center_x <= max_x && center_y <= max_y;
}

} /* namespace automatic_package_measuring */

