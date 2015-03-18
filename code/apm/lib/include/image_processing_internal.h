#ifndef LIB_INCLUDE_IMAGE_PROCESSING_INTERNAL_H_
#define LIB_INCLUDE_IMAGE_PROCESSING_INTERNAL_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

namespace internal {

bool IsContourCentered(std::vector<cv::Point>& contour, const cv::Size& img_size);

extern int CANNY_LOW_THRESHOLD;
extern int CANNY_HIGH_THRESHOLD;
extern double CANNY_RATIO;
extern int CANNY_KERNEL_SIZE;

extern int MORPH_RADIUS;
extern int MORPH_ITERATIONS;

extern double CENTER_THRESHOLD;

extern double POLY_ERROR_TOLERANCE;

extern int HOUGH_RHO;
extern double HOUGH_THETA;
extern int HOUGH_THRESHOLD;
extern int HOUGH_MIN_LENGTH;
extern int HOUGH_MAX_GAP;


} /* namespace automatic_package_measuring::internal */

} /* namespace automatic_package_measuring */

#endif /* LIB_INCLUDE_IMAGE_PROCESSING_INTERNAL_H_ */
