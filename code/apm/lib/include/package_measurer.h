#ifndef INCLUDE_OPENCV2_PACKAGEMEASURER_H_
#define INCLUDE_OPENCV2_PACKAGEMEASURER_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

class PackageMeasurer {
public:
	PackageMeasurer();
	virtual ~PackageMeasurer();

	void AnalyzeImage(cv::Mat& image, int rotation=0);
	void SetReferenceObjectSize(cv::Vec2f dimensions);
	const std::vector<cv::Point2f>& GetReferenceObject() const;
	const std::vector<cv::Point2f>& GetPackage() const;
	const cv::Vec3f& GetMeasurements() const;
	const cv::Vec3i& GetMeasuredEdges() const;

	static const double CANNY_RATIO;
	static const int CANNY_LOW_THRESHOLD;
	static const int CANNY_KERNEL_SIZE;

	static const int MORPH_RADIUS;
	static const int MORPH_ITERATIONS;

	static const double POLY_ERROR_TOLERANCE;

	static const double REFERENCE_OBJECT_MIN_LENGTH;
	static const double PACKAGE_MIN_LENGTH;

	static const double CENTER_THRESHOLD;


private:
	cv::Vec2f reference_object_size;
	cv::Mat RotateImage(const cv::Mat& image, const int rotation);

	std::vector<cv::Point2f> reference_object;
	std::vector<cv::Point2f> package;
	cv::Vec3f measurements;
	cv::Vec3i measured_edges;
};

} /* namespace automatic_package_measuring */

#endif /* INCLUDE_OPENCV2_PACKAGEMEASURER_H_ */
