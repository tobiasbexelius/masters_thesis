#include <opencv2/opencv.hpp>
#include "gtest/gtest.h"
#include "../../lib/include/paper_detection.h"

namespace apm = automatic_package_measuring;
/*
TEST(A4PaperFinderTest, EmptyMat) {
	cv::Mat image;
	std::vector<std::vector<cv::Point>> polygons;
	std::vector<cv::Point> square;
	square.push_back(cv::Point2i(0, 0));
	square.push_back(cv::Point2i(10, 0));
	square.push_back(cv::Point2i(10, 10));
	square.push_back(cv::Point2i(0, 10));
	polygons.push_back(square);

	ASSERT_EQ(std::vector<cv::Point>(), apm::FindPaper(image, polygons));

}

TEST(A4PaperFinderTest, EmptyPolygons) {
	apm::A4PaperFinder paper_finder;

	cv::Mat_<int> image(2, 2);
	std::vector<apm::Polygon> polygons;

	ASSERT_EQ(std::vector<cv::Point>(), paper_finder.findObject(image, polygons));
}

TEST(A4PaperFinderTest, OneEmptyPolygon) {
	apm::A4PaperFinder paper_finder;

	cv::Mat_<int> image(2, 2);
	std::vector<apm::Polygon> polygons;
	apm::Polygon empty;
	polygons.push_back(empty);

	ASSERT_EQ(std::vector<cv::Point>(), paper_finder.findObject(image, polygons));
}

TEST(A4PaperFinderTest, PolygonWithTooFewPoints) {
	apm::A4PaperFinder paper_finder;

	cv::Mat_<int> image(2, 2);
	std::vector<apm::Polygon> polygons;
	apm::Polygon poly;
	poly.push_back(cv::Point2i(0, 0));
	polygons.push_back(poly);

	ASSERT_EQ(std::vector<cv::Point>(), paper_finder.findObject(image, polygons));

	polygons.clear();
	poly.push_back(cv::Point2i(10, 10));
	polygons.push_back(poly);
}
*/
