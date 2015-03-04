#ifndef SRC_MORPHOLOGY_H_
#define SRC_MORPHOLOGY_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

class Morphology {
public:
	Morphology(int element_radius=3, int iterations=6);
	virtual ~Morphology();
	cv::Mat close(cv::Mat image);
	cv::Mat open(cv::Mat image);
	cv::Mat gradient(cv::Mat image);
private:
	cv::Mat morphology(cv::Mat image, int operation);
	int element_radius;
	int iterations;
};

} /* namespace automatic_package_measuring */

#endif /* SRC_MORPHOLOGY_H_ */
