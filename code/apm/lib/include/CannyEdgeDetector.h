#ifndef SRC_EDGEDETECTOR_H_
#define SRC_EDGEDETECTOR_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

class CannyEdgeDetector {
public:
	CannyEdgeDetector(int low_threshold = DEFAULT_LOW_THRESHOLD, int high_threshold = DEFAULT_HIGH_THRESHOLD,
			int kernel_size = DEFAULT_KERNEL_SIZE);
	virtual ~CannyEdgeDetector();
	cv::Mat detectEdges(cv::Mat image);
private:
	int low_threshold;
	int high_threshold;
	int kernel_size;
	static const int DEFAULT_LOW_THRESHOLD;
	static const int DEFAULT_HIGH_THRESHOLD;
	static const int DEFAULT_KERNEL_SIZE;
};

} /* namespace automatic_package_measuring */

#endif /* SRC_EDGEDETECTOR_H_ */
