#ifndef INCLUDE_OPENCV2_PACKAGEMEASURER_H_
#define INCLUDE_OPENCV2_PACKAGEMEASURER_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

class PackageMeasurer {
public:
	PackageMeasurer();
	virtual ~PackageMeasurer();

	void analyzeImage(cv::Mat image);
	std::vector<cv::Point2i>& getReferenceObject();
	std::vector<cv::Point2i>& getPackage();
	cv::Vec3d getMeasurements();

	static const int CANNY_LOW_THRESHOLD;
	static const int MORPH_ELE_RADIUS;
	static const int MORPH_ITERATIONS;
	static const double POLY_ERROR_MARGIN;
	static const double CANNY_RATIO;
	static const double PACKAGE_CONTOUR_MIN_LENGTH;
	static const double PACKAGE_CONTOUR_PERIPHERAL_CONSTRAINT;
private:
	void findReferenceObject();
	void findPackage();
	void measurePackage();

	cv::Mat src;
	cv::Mat src_gray;
	std::vector<cv::Point2i> reference_object;
	std::vector<cv::Point2i> package;
	cv::Vec3d measurements;
	cv::Mat edges;
	std::vector<std::vector<cv::Point2i>> contours;
};

} /* namespace automatic_package_measuring */

#endif /* INCLUDE_OPENCV2_PACKAGEMEASURER_H_ */
