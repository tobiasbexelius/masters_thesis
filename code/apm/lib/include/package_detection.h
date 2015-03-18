#ifndef LIB_INCLUDE_PACKAGE_DETECTION_H_
#define LIB_INCLUDE_PACKAGE_DETECTION_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

void FindPackage(const cv::Mat& image, const cv::Mat& edges, std::vector<cv::Point>& package_out);


} /* namespace automatic_package_measuring */

#endif /* LIB_INCLUDE_PACKAGE_DETECTION_H_ */
