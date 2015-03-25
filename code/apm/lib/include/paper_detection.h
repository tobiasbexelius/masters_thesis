#ifndef LIB_INCLUDE_PAPER_DETECTION_H_
#define LIB_INCLUDE_PAPER_DETECTION_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

std::vector<cv::Point> FindPaper(const cv::Mat& image, const cv::Mat& edges);

}

#endif /* LIB_INCLUDE_PAPER_DETECTION_H_ */
