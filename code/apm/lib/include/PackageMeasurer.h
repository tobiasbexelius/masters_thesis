#ifndef INCLUDE_OPENCV2_PACKAGEMEASURER_H_
#define INCLUDE_OPENCV2_PACKAGEMEASURER_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

class PackageMeasurer {
public:
	PackageMeasurer(int canny_low_threshold = 50, int morph_ele_radius = 1, int morph_iterations = 1,
			double poly_approx_error_margin = 0.02, double canny_ratio = 3.0);
	virtual ~PackageMeasurer();

	void analyzeImage(cv::Mat image);
	std::vector<cv::Point2i>& getReferenceObject();
	std::vector<cv::Point2i>& getPackage();
	cv::Vec3d getMeasurements();
private:
	void findReferenceObject();
	void findPackage();
	void measurePackage();

	cv::Mat src;
	cv::Mat src_gray;
	std::vector<cv::Point2i> reference_object;
	std::vector<cv::Point2i> package;
	cv::Vec3d measurements;
	int canny_low_threshold;
	int morph_ele_radius;
	int morph_iterations;
	double poly_approx_error_margin;
	double canny_ratio;

};

} /* namespace automatic_package_measuring */

#endif /* INCLUDE_OPENCV2_PACKAGEMEASURER_H_ */
