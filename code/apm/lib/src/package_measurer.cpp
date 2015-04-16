#include "../include/paper_detection.h"
#include "../include/package_detection.h"
#include <opencv2/opencv.hpp>

#include "../include/image_processing.h"
#include "../include/package_measurer.h"
#include "../include/package_measuring.h"

namespace automatic_package_measuring {

const double PackageMeasurer::PACKAGE_MIN_LENGTH = 200.0;

PackageMeasurer::PackageMeasurer() {
}

PackageMeasurer::~PackageMeasurer() {
}

void PackageMeasurer::AnalyzeImage(cv::Mat& image) {

	cv::Mat preprocessed_image;
	PreprocessImage(image, preprocessed_image);

	cv::Mat edges;
	FindEdges(preprocessed_image, edges);
	CloseEdges(edges);

	reference_object = FindPaper(preprocessed_image, edges);
	package = FindPackage(preprocessed_image, edges, reference_object);

	measurements = MeasurePackage(image.size(), reference_object, cv::Vec2f(297 / 2.0, 210), package);
}

const std::vector<cv::Point2f>& PackageMeasurer::GetReferenceObject() {
	return reference_object;
}

const std::vector<cv::Point2f>& PackageMeasurer::GetPackage() {
	return package;
}

const cv::Vec3f& PackageMeasurer::GetMeasurements() {
	return measurements;
}

} /* namespace automatic_package_measuring */
