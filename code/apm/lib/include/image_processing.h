#ifndef SRC_CONTOUREXTRACTOR_H_
#define SRC_CONTOUREXTRACTOR_H_

#include <opencv2/opencv.hpp>

namespace automatic_package_measuring {

void PreprocessImage(const cv::Mat& image, cv::Mat& image_out);

void FindEdges(const cv::Mat& image, cv::Mat& edges);

void CloseEdges(cv::Mat& edges);

void FindContours(cv::Mat& edges, bool external_only, std::vector<std::vector<cv::Point>>& contours_out);

void PruneShortContours(std::vector<std::vector<cv::Point>>& contours, double min_length);

void PrunePeripheralContours(std::vector<std::vector<cv::Point>>& contours, const cv::Size& img_size);

void FindConvexPolygons(const std::vector<std::vector<cv::Point>>& contours,
		std::vector<std::vector<cv::Point>>& polygons_out);

void FindConvexPolygon(const std::vector<cv::Point>& contour, std::vector<cv::Point>& polygon_out);

void DetectLines(const cv::Mat& image, std::vector<cv::Vec4i>& lines_out);

void PruneSimilarLines(std::vector<cv::Vec4i> lines);

} /* namespace automatic_package_measuring */

#endif /* SRC_CONTOUREXTRACTOR_H_ */
