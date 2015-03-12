#ifndef SRC_CONTOUREXTRACTOR_H_
#define SRC_CONTOUREXTRACTOR_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

class ContourExtractor {
public:
	ContourExtractor();
	virtual ~ContourExtractor();
	std::vector<std::vector<cv::Point>> extractContours(cv::Mat edges, bool external_only = false);
	void pruneShortContours(std::vector<std::vector<cv::Point>>& contours, double min_length);
	void prunePeripheralContours(std::vector<std::vector<cv::Point>>& contours, const cv::Mat& image,
			double placement_constraint);
private:
	static bool isContourCentered(std::vector<cv::Point>& contour, double placement_constraint,
			int img_height, int img_width);
};

} /* namespace automatic_package_measuring */

#endif /* SRC_CONTOUREXTRACTOR_H_ */
