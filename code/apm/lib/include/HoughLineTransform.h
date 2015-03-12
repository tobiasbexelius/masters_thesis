#ifndef INCLUDE_HOUGHLINETRANSFORM_H_
#define INCLUDE_HOUGHLINETRANSFORM_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

class HoughLineTransform {
public:
	HoughLineTransform(int rho = DEFAULT_RHO, double theta = DEFAULT_THETA, int threshold = DEFAULT_THRESHOLD,
			int min_length = DEFAULT_MIN_LENGTH, int max_gap = DEFAULT_MAX_GAP);
	virtual ~HoughLineTransform();
	std::vector<cv::Vec4i> detectLines(cv::Mat image);

	std::vector<cv::Vec2f> detectLinesNonP(cv::Mat image);

	int rho;
	double theta;
	int threshold;
	int min_length;
	int max_gap;

	static const int DEFAULT_RHO;
	static const double DEFAULT_THETA;
	static const int DEFAULT_THRESHOLD;
	static const int DEFAULT_MIN_LENGTH;
	static const int DEFAULT_MAX_GAP;
};

} /* namespace automatic_package_measuring */

#endif /* INCLUDE_HOUGHLINETRANSFORM_H_ */
