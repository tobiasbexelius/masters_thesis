#ifndef LIB_INCLUDE_PAPER_DETECTION_INTERNAL_H_
#define LIB_INCLUDE_PAPER_DETECTION_INTERNAL_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

namespace internal {

bool IsColorOK(cv::Mat image, std::vector<cv::Point> polygon);
bool IsShapeOK(std::vector<cv::Point> polygon);
bool IsSizeOK(cv::Size img_size, std::vector<cv::Point> polygon);
double euclideanDistance(cv::Point p1, cv::Point p2);
extern int A4_LONG_SIDE;
extern int A4_SHORT_SIDE;
extern double MIN_IMAGE_AREA;
extern double MIN_PAPER_CONTOUR_LENGTH;

}

}

#endif /* LIB_INCLUDE_PAPER_DETECTION_INTERNAL_H_ */
