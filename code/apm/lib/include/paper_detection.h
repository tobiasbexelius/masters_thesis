#ifndef LIB_INCLUDE_PAPER_DETECTION_H_
#define LIB_INCLUDE_PAPER_DETECTION_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

void FindPaper(const cv::Mat& image, const cv::Mat& edges, std::vector<cv::Point>& paper_out);

}

#endif /* LIB_INCLUDE_PAPER_DETECTION_H_ */
