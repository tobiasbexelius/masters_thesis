#include <opencv2/opencv.hpp>

#include "../../lib/include/image_processing.h"
#include "gtest/gtest.h"

namespace apm = automatic_package_measuring;

TEST(TestPruneContours, TooShortContour) {
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Point> short_contour = { cv::Point2i(0, 0), cv::Point2i(0, 2), cv::Point2i(2, 2),
			cv::Point2i(2, 0) };
	contours.push_back(short_contour);
	apm::PruneShortContours(contours, 10);

	ASSERT_TRUE(contours.empty());
}

TEST(TestPruneContours, LongEnoughContour) {
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Point> long_contour = { cv::Point2i(0, 0), cv::Point2i(0, 4), cv::Point2i(4, 4),
			cv::Point2i(4, 0) };
	contours.push_back(long_contour);
	apm::PruneShortContours(contours, 10);

	ASSERT_EQ(1, contours.size());
}

TEST(TestPruneContours, TwoShortAndOneLongContour) {
	std::vector<std::vector<cv::Point>> contours;

	std::vector<cv::Point> short_contour = { cv::Point2i(0, 0), cv::Point2i(0, 2), cv::Point2i(2, 2),
			cv::Point2i(2, 0) };

	std::vector<cv::Point> short_contour2 = { cv::Point2i(0, 0), cv::Point2i(0, 3), cv::Point2i(3, 3),
			cv::Point2i(3, 0) };

	std::vector<cv::Point> long_contour = { cv::Point2i(0, 0), cv::Point2i(0, 4), cv::Point2i(4, 4),
			cv::Point2i(4, 0) };

	contours.push_back(short_contour);
	contours.push_back(short_contour2);
	contours.push_back(long_contour);

	apm::PruneShortContours(contours, 10);

	ASSERT_EQ(1, contours.size());
}

TEST(TestPrunePerpheralContours, CenteredContour) {
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Point> centered_contour = { cv::Point2i(3, 3), cv::Point2i(3, 7), cv::Point2i(7, 7),
			cv::Point2i(7, 3) };
	contours.push_back(centered_contour);
	cv::Mat_<int> image(10, 10);
	apm::PrunePeripheralContours(contours, image.size());

	ASSERT_EQ(1, contours.size());
}

TEST(TestPrunePerpheralContours, PeripheralContour) {
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Point> peripheral_contour = { cv::Point2i(0, 0), cv::Point2i(3, 0), cv::Point2i(3, 3),
			cv::Point2i(0, 3) };
	contours.push_back(peripheral_contour);
	cv::Mat_<int> image(15, 15);
	apm::PrunePeripheralContours(contours, image.size());

	ASSERT_TRUE(contours.empty());
}

TEST(TestPrunePerpheralContours, BarelyCenteredContour) {
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Point> centered_contour = { cv::Point2i(0, 0), cv::Point2i(6, 0), cv::Point2i(6, 6),
			cv::Point2i(0, 6) };
	contours.push_back(centered_contour);
	cv::Mat_<int> image(10, 10);
	apm::PrunePeripheralContours(contours, image.size());

	ASSERT_EQ(1, contours.size());
}

