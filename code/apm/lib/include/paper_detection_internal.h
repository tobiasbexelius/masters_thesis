#ifndef LIB_INCLUDE_PAPER_DETECTION_INTERNAL_H_
#define LIB_INCLUDE_PAPER_DETECTION_INTERNAL_H_

#include <unordered_map>
#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

namespace internal {

double RatePaper(const cv::Mat& image, const std::vector<cv::Vec4i>& lines, const std::vector<cv::Point2f>& paper);
std::vector<std::vector<cv::Point2f>> FindPapers(const std::vector<cv::Vec4i>& lines,
		const cv::Size& image_size);

extern double A4_LONG_SIDE;
extern double A4_SHORT_SIDE;
extern double A4_LENGTH_RATIO;
extern double MIN_PAPER_PARALLEL_LINE_DIST;
extern double MIN_PAPER_CONTOUR_LENGTH;
extern int MAX_PAPER_LINE_PAIRS;
extern double MIN_PAPER_ACCEPTED_SCORE;
extern double MIN_PAPER_ACCEPTED_SUBSCORE;
extern double MAX_PAPER_PARALLEL_ANGLE;
extern int NUM_BINS;
} /* namespace automatic_package_measuring::internal */

} /* namespace automatic_package_measuring */

#endif /* LIB_INCLUDE_PAPER_DETECTION_INTERNAL_H_ */
