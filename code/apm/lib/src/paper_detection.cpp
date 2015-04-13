#include "../include/paper_detection.h"
#include "../include/paper_detection_internal.h"
#include "../include/image_processing.h"
#include <queue>
#include <cmath>

using namespace automatic_package_measuring::internal;

namespace automatic_package_measuring {

std::vector<cv::Point2f> FindPaper(const cv::Mat& image, const cv::Mat& edges) {
	if (image.empty() || edges.empty()) {
		return std::vector<cv::Point2f>();
	}
	cv::Mat edges_cpy = edges.clone();
	std::vector<std::vector<cv::Point>> contours;
	FindContours(edges_cpy, false, contours);
	if (contours.empty())
		return std::vector<cv::Point2f>();
	PruneShortContours(contours, MIN_PAPER_CONTOUR_LENGTH);
	PrunePeripheralContours(contours, image.size());
	std::vector<std::vector<cv::Point>> polygons;
	FindConvexPolygons(contours, polygons);
	for (std::vector<cv::Point> polygon : polygons) {
		if (polygon.empty()) {
			continue;
		}
		if (IsSizeOK(image.size(), polygon) && IsShapeOK(polygon) && IsColorOK(image, polygon)) {
			std::vector<cv::Point2f> paper;
			for(auto point : polygon)
				paper.push_back(cv::Point2f(point.x,point.y));
			return paper;
		}
	}

	return std::vector<cv::Point2f>();
}

namespace internal {

bool IsSizeOK(cv::Size img_size, std::vector<cv::Point> polygon) {
	double area = cv::contourArea(polygon, false);
	double total_area = img_size.height * img_size.width;
	return area / total_area >= MIN_IMAGE_AREA;
}

bool IsColorOK(cv::Mat image, std::vector<cv::Point> polygon) {
	if (image.type() != CV_8UC1)
		cv::cvtColor(image, image, CV_BGR2GRAY);

	cv::Rect bounding_rect = cv::boundingRect(polygon);
	cv::equalizeHist(image, image);
	cv::Mat roi = image(bounding_rect);
	cv::Mat histogram;
	int channels[] = { 0 };
	int bins[] = { 10 };
	const float *ranges[1];
	float range[] = { 0.0f, 255.0f };
	ranges[0] = range;
	cv::calcHist(&roi, 1, channels, cv::Mat(), histogram, 1, bins, ranges);

	double top_bin = histogram.at<float>(bins[0] - 1);//s + histogram.at<float>(bins[0] - 2)
			;//+ histogram.at<float>(bins[0] - 3);

	return top_bin / ((double) (roi.rows * roi.cols)) > 0.5;
}

bool IsShapeOK(std::vector<cv::Point> polygon) {
	if (polygon.size() < 4) // don't accept triangles, lines or points
		return false;
	std::priority_queue<double> edges;
	auto previous = polygon.begin();
	cv::Point first = *previous;
	auto current = ++polygon.begin();

	for (; current != polygon.end(); ++previous, ++current) {
		edges.push(cv::norm(*previous - *current));
	}
	edges.push(cv::norm(*previous - first));
	double four_edges_length = 0;
	for (int i = 0; i < 4; ++i) {
		four_edges_length += edges.top();
		edges.pop();
	}

	double total_length = cv::arcLength(polygon, true);

	return four_edges_length / total_length >= 0.95;
}

int A4_LONG_SIDE = 297;
int A4_SHORT_SIDE = 210;
double MIN_IMAGE_AREA = 0.01;
double MIN_PAPER_CONTOUR_LENGTH = 20.0;

} /* namespace internal */

} /* namespace automatic_package_measuring */
