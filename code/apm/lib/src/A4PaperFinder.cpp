#include "../include/A4PaperFinder.h"
#include <queue>
#include <cmath>

namespace automatic_package_measuring {

const int A4PaperFinder::LONG_SIDE = 297;
const int A4PaperFinder::SHORT_SIDE = 210;
const double A4PaperFinder::MIN_SIZE = 0.01;

A4PaperFinder::A4PaperFinder() {
}

A4PaperFinder::~A4PaperFinder() {
}

std::vector<cv::Point> A4PaperFinder::findObject(cv::Mat image,
		std::vector<Polygon> polygons) {
	if (image.empty()) {
#if DEBUG
		std::cerr << "Error in A4PaperFinder: empty image received." << std::endl;
#endif
		return std::vector<cv::Point>();
	}

	for (std::vector<cv::Point> polygon : polygons) {
		if (polygon.empty()) {
#if DEBUG
			std::cerr << "Error in A4PaperFinder: empty polygon received." << std::endl;
#endif
			continue;
		}

		if (isSizeOK(image, polygon) && isShapeOK(polygon)
				&& isColorOK(image, polygon))
			return polygon;
	}

	return std::vector<cv::Point>();
}

bool A4PaperFinder::isSizeOK(cv::Mat image, Polygon polygon) {
	double area = cv::contourArea(polygon, false);
	double total_area = image.rows * image.cols;
	return area / total_area >= MIN_SIZE;
}

bool A4PaperFinder::isColorOK(cv::Mat image, Polygon polygon) {
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

	double top_three_bins = histogram.at<float>(bins[0] - 1)
			+ histogram.at<float>(bins[0] - 2)
			+ histogram.at<float>(bins[0] - 3);

	return top_three_bins / ((double) (roi.rows * roi.cols));
}

bool A4PaperFinder::isShapeOK(Polygon polygon) {
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

} /* namespace automatic_package_measuring */
