#include <assert.h>
#include "../include/image_processing.h"
#include "../include/image_processing_internal.h"

using namespace automatic_package_measuring::internal;

namespace automatic_package_measuring {

void PreprocessImage(const cv::Mat& image, cv::Mat& image_out) {

#if 0
	cv::Mat filtered_image;
	cv::bilateralFilter(image, filtered_image, 5, 50, 50);
	cv::cvtColor(filtered_image, image_out, CV_BGR2GRAY);
#endif

#if 1
	cv::Mat image_gray;
	cv::cvtColor(image, image_gray, CV_BGR2GRAY);
	cv::medianBlur(image_gray, image_out, BLUR_KERNEL_SIZE);
#endif

}

void FindEdges(const cv::Mat& image, cv::Mat& edges) {
	cv::Canny(image, edges, CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD, CANNY_KERNEL_SIZE);
}

void CloseEdges(cv::Mat& edges) {
	cv::Size morph_size = cv::Size(2 * MORPH_RADIUS + 1, 2 * MORPH_RADIUS + 1);
	cv::Point anchor = cv::Point(MORPH_RADIUS, MORPH_RADIUS);
	cv::Mat structuring_element = cv::getStructuringElement(cv::MORPH_RECT, morph_size, anchor);
	cv::morphologyEx(edges, edges, cv::MORPH_CLOSE, structuring_element, anchor, MORPH_ITERATIONS);
}

void FindContours(cv::Mat& edges, bool external_only, std::vector<std::vector<cv::Point>>& contours_out) {

	int mode;
	if (external_only)
		mode = CV_RETR_EXTERNAL;
	else
		mode = CV_RETR_LIST;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(edges, contours_out, hierarchy, mode, CV_CHAIN_APPROX_SIMPLE);
}

void PruneShortContours(std::vector<std::vector<cv::Point> >& contours, double min_length) {

	contours.erase(
			std::remove_if(contours.begin(), contours.end(), [min_length](std::vector<cv::Point>& contour) {
				return cv::arcLength(contour, false) < min_length;}), contours.end());
}

void PrunePeripheralContours(std::vector<std::vector<cv::Point> >& contours, const cv::Size& img_size) {

	contours.erase(
			std::remove_if(contours.begin(), contours.end(), [img_size](std::vector<cv::Point>& contour) {
				return !IsContourCentered(contour, img_size);}), contours.end());
}

void FindConvexPolygons(const std::vector<std::vector<cv::Point>>& contours,
		std::vector<std::vector<cv::Point>>& polygons_out) {
	for (auto contour : contours) {
		std::vector<cv::Point> polygon;
		FindConvexPolygon(contour, polygon);
		if (!polygon.empty())
			polygons_out.push_back(polygon);
	}
}

void FindConvexPolygon(const std::vector<cv::Point>& contour, std::vector<cv::Point>& polygon_out) {
	double arc_length = cv::arcLength(contour, true);
	cv::approxPolyDP(contour, polygon_out, arc_length * POLY_ERROR_TOLERANCE, true);
	if (!cv::isContourConvex(polygon_out))
		polygon_out.clear();
}

void DetectLines(const cv::Mat& image, std::vector<cv::Vec4i>& lines_out) {
	cv::HoughLinesP(image, lines_out, HOUGH_RHO, HOUGH_THETA, HOUGH_THRESHOLD, HOUGH_MIN_LENGTH,
			HOUGH_MAX_GAP);
}

namespace internal {

bool IsContourCentered(std::vector<cv::Point>& contour, const cv::Size& img_size) {

	assert(CENTER_THRESHOLD >= 0 && CENTER_THRESHOLD <= 1);

	cv::Rect bounding_rect = cv::boundingRect(contour);

	int center_x = bounding_rect.x + bounding_rect.width / 2;
	int center_y = bounding_rect.y + bounding_rect.height / 2;

	int min_x = img_size.width * CENTER_THRESHOLD;
	int min_y = img_size.height * CENTER_THRESHOLD;

	int max_x = img_size.width * (1 - CENTER_THRESHOLD);
	int max_y = img_size.height * (1 - CENTER_THRESHOLD);

	return center_x >= min_x && center_y >= min_y && center_x <= max_x && center_y <= max_y;
}

int BLUR_KERNEL_SIZE = 9;

int CANNY_LOW_THRESHOLD = 10;
double CANNY_RATIO = 3.0;
int CANNY_KERNEL_SIZE = 3;
int CANNY_HIGH_THRESHOLD = CANNY_LOW_THRESHOLD * CANNY_RATIO;

int MORPH_RADIUS = 2;
int MORPH_ITERATIONS = 2;

double CENTER_THRESHOLD = 0.25;

double POLY_ERROR_TOLERANCE = 0.02;

int HOUGH_RHO = 2;
double HOUGH_THETA = CV_PI / 180;
int HOUGH_THRESHOLD = 50;
int HOUGH_MIN_LENGTH = 30;//80;
int HOUGH_MAX_GAP = 55;

} /* namespace automatic_package_measuring::internal */

} /* namespace automatic_package_measuring */

