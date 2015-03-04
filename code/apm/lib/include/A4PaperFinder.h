#ifndef SRC_A4PAPERFINDER_H_
#define SRC_A4PAPERFINDER_H_

#include "ReferenceObjectFinder.h"
#include <opencv2/opencv.hpp>

namespace automatic_package_measuring{

class A4PaperFinder : ReferenceObjectFinder{
public:
	A4PaperFinder();
	virtual ~A4PaperFinder();
	std::vector<cv::Point> findObject(cv::Mat image, std::vector<std::vector<cv::Point>> polygons);
private:
	bool isColorOK(cv::Mat image, Polygon polygon);
	bool isShapeOK(Polygon polygon);
	bool isSizeOK(cv::Mat image, Polygon polygon);
	double euclideanDistance(cv::Point p1, cv::Point p2);
	static const int LONG_SIDE;
	static const int SHORT_SIDE;
	static const double MIN_SIZE;
};

} /* namespace automatic_package_measuring */

#endif /* SRC_A4PAPERFINDER_H_ */
