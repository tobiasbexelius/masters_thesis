#ifndef LIB_INCLUDE_PACKAGE_DETECTION_H_
#define LIB_INCLUDE_PACKAGE_DETECTION_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

std::vector<cv::Point2f> FindPackage(const cv::Mat& image, const cv::Mat& edges, std::vector<cv::Point2f>& reference_object);


} /* namespace automatic_package_measuring */

#endif /* LIB_INCLUDE_PACKAGE_DETECTION_H_ */
