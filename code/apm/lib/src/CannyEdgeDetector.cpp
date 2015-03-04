#include "../include/CannyEdgeDetector.h"

namespace automatic_package_measuring {

using cv::Mat;

const int CannyEdgeDetector::DEFAULT_LOW_THRESHOLD = 70;
const int CannyEdgeDetector::DEFAULT_HIGH_THRESHOLD = 3 * DEFAULT_LOW_THRESHOLD;
const int CannyEdgeDetector::DEFAULT_KERNEL_SIZE = 3;

CannyEdgeDetector::~CannyEdgeDetector() {
}

CannyEdgeDetector::CannyEdgeDetector(int low_threshold, int high_threshold,
		int kernel_size) :
		low_threshold(low_threshold), high_threshold(high_threshold), kernel_size(
				kernel_size) {
}

Mat CannyEdgeDetector::detectEdges(Mat grayscale_image) {

	Mat edges;

	cv::blur(grayscale_image, edges, cv::Size(3,3));

	cv::Canny(edges, edges, low_threshold, high_threshold, kernel_size);

	return edges;
}

} /* namespace automatic_package_measuring */
