#include "../include/paper_detection.h"
#include "../include/package_detection.h"
#include <opencv2/opencv.hpp>

#include "../include/image_processing.h"
#include "../include/package_measurer.h"
#include "../include/package_measuring.h"
//#include <android/log.h>
//#define LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
//#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
//#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
//#define LOG_TAG "APM"
#define LOGD printf
namespace automatic_package_measuring {

//const double PackageMeasurer::PACKAGE_MIN_LENGTH = 200.0;

PackageMeasurer::PackageMeasurer() {
}

PackageMeasurer::~PackageMeasurer() {
}

void PackageMeasurer::AnalyzeImage(cv::Mat& image, int rotation) {

	cv::Mat rotated_image = RotateImage(image, rotation);
	rotated_size = rotated_image.size();

	cv::Mat preprocessed_image;
	PreprocessImage(rotated_image, preprocessed_image);

	cv::Mat edges;
	FindEdges(preprocessed_image, edges);
	CloseEdges(edges);

	reference_object = FindPaper(preprocessed_image, edges);

	package = FindPackage(preprocessed_image, edges, reference_object);
	measured_edges = cv::Vec3i();

	measurements = MeasurePackage(preprocessed_image.size(), reference_object, reference_object_size, package,
			measured_edges);

	LOGD("MEASUREMENTS: %f, %f, %f", measurements[0], measurements[1], measurements[2]);
}

void PackageMeasurer::SetReferenceObjectSize(cv::Vec2f dimensions) {
	reference_object_size = dimensions;

}

double PackageMeasurer::GetCalibHeight() const {
	cv::Vec3i edges_tmp;
	cv::Vec3f res = MeasurePackage(rotated_size, reference_object, reference_object_size, package, edges_tmp, false);
	return res[2];
}

cv::Mat PackageMeasurer::RotateImage(const cv::Mat& image, const int rotation) {
	cv::Mat rotated_image;
	switch (rotation) {
	case 90:
		cv::transpose(image, rotated_image);
		cv::flip(rotated_image, rotated_image, 1);
		break;
	case 180:
		cv::flip(image, rotated_image, -1);
		break;
	case 270:
		cv::transpose(image, rotated_image);
		cv::flip(rotated_image, rotated_image, 0);
		break;
	case 0:
	default:
		rotated_image = image;
		break;
	}

	return rotated_image;
}

const std::vector<cv::Point2f>& PackageMeasurer::GetReferenceObject() const {
	return reference_object;
}

const std::vector<cv::Point2f>& PackageMeasurer::GetPackage() const {
	return package;
}

const cv::Vec3f& PackageMeasurer::GetMeasurements() const {
	return measurements;
}

const cv::Vec3i& PackageMeasurer::GetMeasuredEdges() const {
	return measured_edges;
}

} /* namespace automatic_package_measuring */
