#ifndef LIB_INCLUDE_PACKAGE_MEASURING_H_
#define LIB_INCLUDE_PACKAGE_MEASURING_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

cv::Vec3f MeasurePackage(const cv::Size& img_size, const std::vector<cv::Point2f>& reference_object,
		const cv::Vec2f ref_dimensions, const std::vector<cv::Point2f>& package,
		cv::Vec3i& measured_edges_out, bool auto_calibrate=true);
cv::Vec3f MeasurePackageAngle(const cv::Size& img_size, const std::vector<cv::Point2f>& reference_object,
		const cv::Vec2f ref_dimensions, const std::vector<cv::Point2f>& package, bool auto_calibrate);
cv::Vec3f MeasurePackageStraight(const cv::Size& img_size, const std::vector<cv::Point2f>& reference_object,
		const cv::Vec2f ref_dimensions, const std::vector<cv::Point2f>& package, bool auto_calibrate);

} /* namespace automatic_package_measuring*/

#endif /* LIB_INCLUDE_PACKAGE_MEASURING_H_ */
