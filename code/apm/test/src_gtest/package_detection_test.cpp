#include <opencv2/opencv.hpp>
#include "gtest/gtest.h"
#include "../../lib/include/package_detection.h"
#include "../../lib/include/package_detection_internal.h"

namespace apm = automatic_package_measuring;

TEST(LineIntersectionTest, SegmentIntersection) {
	cv::Point2f p1(20, 10);
	cv::Point2f p2(10, 10);

	cv::Point2f p3(15, 5);
	cv::Point2f p4(15, 15);

	cv::Point2f intersection;
	bool do_lines_intersect = apm::internal::LineIntersection(p1, p2, p3, p4, intersection);

	ASSERT_TRUE(do_lines_intersect);
	ASSERT_EQ(cv::Point2f(15, 10), intersection);
}

TEST(LineIntersectionTest, ExtendedSegmentIntersection) {
	cv::Point2f p1(0, 0);
	cv::Point2f p2(5, 5);

	cv::Point2f p3(0, 20);
	cv::Point2f p4(20, 0);

	cv::Point2f intersection;
	bool do_lines_intersect = apm::internal::LineIntersection(p1, p2, p3, p4, intersection);

	ASSERT_TRUE(do_lines_intersect);
	ASSERT_EQ(cv::Point2f(10, 10), intersection);
}

TEST(LineIntersectionTest, NoIntersection) {
	cv::Point2f p1(0, 0);
	cv::Point2f p2(10, 0);

	cv::Point2f p3(0, 10);
	cv::Point2f p4(10, 10);

	cv::Point2f intersection;
	bool do_lines_intersect = apm::internal::LineIntersection(p1, p2, p3, p4, intersection);

	ASSERT_FALSE(do_lines_intersect);
}

TEST(LineIntersectionTest, ParallellLines) {
	cv::Point2f p1(0, 0);
	cv::Point2f p2(5, 5);

	cv::Point2f p3(10, 10);
	cv::Point2f p4(20, 20);

	cv::Point2f intersection;
	bool do_lines_intersect = apm::internal::LineIntersection(p1, p2, p3, p4, intersection);

	ASSERT_FALSE(do_lines_intersect);
}

TEST(LineIntersectionTest, IntersectionInEndPoint) {
	cv::Point2f p1(0, 5);
	cv::Point2f p2(5, 5);

	cv::Point2f p3(5, 0);
	cv::Point2f p4(5, 5);

	cv::Point2f intersection;
	bool do_lines_intersect = apm::internal::LineIntersection(p1, p2, p3, p4, intersection);

	ASSERT_TRUE(do_lines_intersect);
	ASSERT_EQ(cv::Point2f(5, 5), intersection);
}

TEST(FindCornersTest, PerfectSquare) {
	std::vector<cv::Vec4i> lines;
	lines.push_back(cv::Vec4i(5, 5, 5, 10));
	lines.push_back(cv::Vec4i(5, 10, 10, 10));
	lines.push_back(cv::Vec4i(10, 10, 10, 5));
	lines.push_back(cv::Vec4i(10, 5, 5, 5));
	std::vector<cv::Point2i> corners;

	double avg_dimension = 300;
	apm::internal::FindCorners(lines, corners, 20, avg_dimension * 0.15);

	ASSERT_EQ(4, corners.size());
	ASSERT_NE(corners.end(), std::find(corners.begin(), corners.end(), cv::Point2i(5, 5)));
	ASSERT_NE(corners.end(), std::find(corners.begin(), corners.end(), cv::Point2i(5, 10)));
	ASSERT_NE(corners.end(), std::find(corners.begin(), corners.end(), cv::Point2i(10, 10)));
	ASSERT_NE(corners.end(), std::find(corners.begin(), corners.end(), cv::Point2i(10, 5)));
}

TEST(FindCornersTest, SquareWithOutliers) {
	std::vector<cv::Vec4i> lines;
	lines.push_back(cv::Vec4i(0, 0, 90, 0));
	lines.push_back(cv::Vec4i(100, 0, 100, 103));
	lines.push_back(cv::Vec4i(100, 100, 5, 100));
	lines.push_back(cv::Vec4i(0, 100, 0, 10));
	lines.push_back(cv::Vec4i(120, 50, 200, 50));
	std::vector<cv::Point2i> corners;

	double avg_dimension = 300;
	apm::internal::FindCorners(lines, corners, 20, avg_dimension * 0.15);

	ASSERT_EQ(4, corners.size());
	ASSERT_NE(corners.end(), std::find(corners.begin(), corners.end(), cv::Point2i(0, 0)));
	ASSERT_NE(corners.end(), std::find(corners.begin(), corners.end(), cv::Point2i(100, 0)));
	ASSERT_NE(corners.end(), std::find(corners.begin(), corners.end(), cv::Point2i(100, 100)));
	ASSERT_NE(corners.end(), std::find(corners.begin(), corners.end(), cv::Point2i(0, 100)));
}

TEST(FindCornersTest, NonPerpendicular) {
	std::vector<cv::Vec4i> lines;
	lines.push_back(cv::Vec4i(0, 0, 90, 0));
	lines.push_back(cv::Vec4i(100, 0, 90, 90));
	lines.push_back(cv::Vec4i(100, 100, 5, 100));
	lines.push_back(cv::Vec4i(10, 90, 0, 10));
	std::vector<cv::Point2i> corners;

	double avg_dimension = 300;
	apm::internal::FindCorners(lines, corners, 20, avg_dimension * 0.15);

	ASSERT_EQ(4, corners.size());
}

