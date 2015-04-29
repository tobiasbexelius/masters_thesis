#ifndef LIB_INCLUDE_PACKAGE_MEASURING_H_
#define LIB_INCLUDE_PACKAGE_MEASURING_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

cv::Vec3f MeasurePackage(const cv::Size& img_size, const std::vector<cv::Point2f>& reference_object, const cv::Vec2f ref_dimensions,
		const std::vector<cv::Point2f>& package, cv::Vec3i& measured_edges_out);
cv::Vec3f UncalibMeasurePackage(const cv::Size& img_size, const std::vector<cv::Point2f>& reference_object, const cv::Vec2f ref_dimensions,
		const std::vector<cv::Point2f>& package, cv::Vec3i& measured_edges_out);


} /* namespace automatic_package_measuring*/

#endif /* LIB_INCLUDE_PACKAGE_MEASURING_H_ */
