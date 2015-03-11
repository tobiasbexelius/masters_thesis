#include <opencv2/opencv.hpp>
#include "gtest/gtest.h"
#include "../../lib/include/ContourExtractor.h"

namespace apm = automatic_package_measuring;

TEST(TestPruneContours, TooShortContour) {
	apm::ContourExtractor contour_extractor;
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Point> short_contour = { cv::Point2i(0, 0), cv::Point2i(0, 2),
			cv::Point2i(2, 2), cv::Point2i(2, 0) };
	contours.push_back(short_contour);
	contour_extractor.pruneShortContours(contours, 10);

	ASSERT_TRUE(contours.empty());
}

TEST(TestPruneContours, LongEnoughContour) {
	apm::ContourExtractor contour_extractor;
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Point> long_contour = { cv::Point2i(0, 0), cv::Point2i(0, 4),
			cv::Point2i(4, 4), cv::Point2i(4, 0) };
	contours.push_back(long_contour);
	contour_extractor.pruneShortContours(contours, 10);

	ASSERT_EQ(1, contours.size());
}

TEST(TestPruneContours, TwoShortAndOneLongContour) {
	apm::ContourExtractor contour_extractor;
	std::vector<std::vector<cv::Point>> contours;

	std::vector<cv::Point> short_contour = { cv::Point2i(0, 0), cv::Point2i(0, 2),
			cv::Point2i(2, 2), cv::Point2i(2, 0) };

	std::vector<cv::Point> short_contour2 = { cv::Point2i(0, 0), cv::Point2i(0, 3),
				cv::Point2i(3, 3), cv::Point2i(3, 0) };

	std::vector<cv::Point> long_contour = { cv::Point2i(0, 0), cv::Point2i(0, 4),
			cv::Point2i(4, 4), cv::Point2i(4, 0) };

	contours.push_back(short_contour);
	contours.push_back(short_contour2);
	contours.push_back(long_contour);

	contour_extractor.pruneShortContours(contours, 10);

	ASSERT_EQ(1, contours.size());
}

TEST(TestPrunePerpheralContours, CenteredContour) {
	apm::ContourExtractor contour_extractor;
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Point> centered_contour = { cv::Point2i (3,3), cv::Point2i (3,7), cv::Point2i (7,7), cv::Point2i (7,3)};
	contours.push_back(centered_contour);
	cv::Mat_<int> image(10, 10);
	contour_extractor.prunePeripheralContours(contours, image, 0.25);

	ASSERT_EQ(1, contours.size());
}

TEST(TestPrunePerpheralContours, PeripheralContour) {
	apm::ContourExtractor contour_extractor;
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Point> peripheral_contour = { cv::Point2i (0,0), cv::Point2i (3,0), cv::Point2i (3,3), cv::Point2i (0,3)};
	contours.push_back(peripheral_contour);
	cv::Mat_<int> image(10, 10);
	contour_extractor.prunePeripheralContours(contours, image, 0.33);

	ASSERT_TRUE(contours.empty());
}

TEST(TestPrunePerpheralContours, BarelyCenteredContour) {
	apm::ContourExtractor contour_extractor;
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Point> centered_contour = { cv::Point2i (0,0), cv::Point2i (6,0), cv::Point2i (6,6), cv::Point2i (0,6)};
	contours.push_back(centered_contour);
	cv::Mat_<int> image(10, 10);
	contour_extractor.prunePeripheralContours(contours, image, 0.3);

	ASSERT_EQ(1, contours.size());
}

