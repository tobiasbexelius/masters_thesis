#include <opencv2/opencv.hpp>
#include "gtest/gtest.h"
#include "../../lib/include/package_detection.h"
#include "../../lib/include/package_detection_internal.h"
#include <unordered_map>
namespace apm = automatic_package_measuring;
// std::unordered_map<int, std::vector<int>> FindNeighbouringLines(const std::vector<cv::Vec4i>& lines,
//std::unordered_map<int, int> line_to_pair) {
/*TEST(NieghbouringLinesTest, BasicTest) {
	cv::Vec4i l0(0, 5, 0, 15);
	cv::Vec4i l1(0, 15, 10, 15);
	cv::Vec4i l2(10, 15, 15, 10);
	cv::Vec4i l3(15, 10, 15, 0);
	cv::Vec4i l4(15, 0, 5, 0);
	cv::Vec4i l5(5, 0, 0, 5);
	std::vector<cv::Vec4i> lines = { l0, l1, l2, l3, l4, l5 };

	std::unordered_map<int, int> line_to_pair;
	line_to_pair[0] = 0;
	line_to_pair[3] = 0;
	line_to_pair[1] = 1;
	line_to_pair[4] = 1;
	line_to_pair[2] = 2;
	line_to_pair[5] = 2;

	std::unordered_map<int, std::vector<int>> neighbour_list = apm::internal::FindNeighbouringLines(lines,
			line_to_pair);

	ASSERT_EQ(6, neighbour_list.size());

	ASSERT_NE(neighbour_list[0].end(), std::find(neighbour_list[0].begin(), neighbour_list[0].end(), 1));
	ASSERT_NE(neighbour_list[0].end(), std::find(neighbour_list[0].begin(), neighbour_list[0].end(), 5));

	ASSERT_NE(neighbour_list[1].end(), std::find(neighbour_list[1].begin(), neighbour_list[1].end(), 0));
	ASSERT_NE(neighbour_list[1].end(), std::find(neighbour_list[1].begin(), neighbour_list[1].end(), 2));

	ASSERT_NE(neighbour_list[2].end(), std::find(neighbour_list[2].begin(), neighbour_list[2].end(), 3));
	ASSERT_NE(neighbour_list[2].end(), std::find(neighbour_list[2].begin(), neighbour_list[2].end(), 1));

	ASSERT_NE(neighbour_list[3].end(), std::find(neighbour_list[3].begin(), neighbour_list[3].end(), 4));
	ASSERT_NE(neighbour_list[3].end(), std::find(neighbour_list[3].begin(), neighbour_list[3].end(), 2));

	ASSERT_NE(neighbour_list[4].end(), std::find(neighbour_list[4].begin(), neighbour_list[4].end(), 5));
	ASSERT_NE(neighbour_list[4].end(), std::find(neighbour_list[4].begin(), neighbour_list[4].end(), 3));

	ASSERT_NE(neighbour_list[5].end(), std::find(neighbour_list[5].begin(), neighbour_list[5].end(), 0));
	ASSERT_NE(neighbour_list[5].end(), std::find(neighbour_list[5].begin(), neighbour_list[5].end(), 4));

}*/

//TEST(LineIntersectionTest, SegmentIntersection) {
//	cv::Point2f p1(20, 10);
//	cv::Point2f p2(10, 10);
//
//	cv::Point2f p3(15, 5);
//	cv::Point2f p4(15, 15);
//
//	cv::Point2f intersection;
//	bool do_lines_intersect = apm::internal::LineIntersection(p1, p2, p3, p4, intersection);
//
//	ASSERT_TRUE(do_lines_intersect);
//	ASSERT_EQ(cv::Point2f(15, 10), intersection);
//}
//
//TEST(LineIntersectionTest, ExtendedSegmentIntersection) {
//	cv::Point2f p1(0, 0);
//	cv::Point2f p2(5, 5);
//
//	cv::Point2f p3(0, 20);
//	cv::Point2f p4(20, 0);
//
//	cv::Point2f intersection;
//	bool do_lines_intersect = apm::internal::LineIntersection(p1, p2, p3, p4, intersection);
//
//	ASSERT_TRUE(do_lines_intersect);
//	ASSERT_EQ(cv::Point2f(10, 10), intersection);
//}
//
//TEST(LineIntersectionTest, NoIntersection) {
//	cv::Point2f p1(0, 0);
//	cv::Point2f p2(10, 0);
//
//	cv::Point2f p3(0, 10);
//	cv::Point2f p4(10, 10);
//
//	cv::Point2f intersection;
//	bool do_lines_intersect = apm::internal::LineIntersection(p1, p2, p3, p4, intersection);
//
//	ASSERT_FALSE(do_lines_intersect);
//}
//
//TEST(LineIntersectionTest, ParallellLines) {
//	cv::Point2f p1(0, 0);
//	cv::Point2f p2(5, 5);
//
//	cv::Point2f p3(10, 10);
//	cv::Point2f p4(20, 20);
//
//	cv::Point2f intersection;
//	bool do_lines_intersect = apm::internal::LineIntersection(p1, p2, p3, p4, intersection);
//
//	ASSERT_FALSE(do_lines_intersect);
//}
//
//TEST(LineIntersectionTest, IntersectionInEndPoint) {
//	cv::Point2f p1(0, 5);
//	cv::Point2f p2(5, 5);
//
//	cv::Point2f p3(5, 0);
//	cv::Point2f p4(5, 5);
//
//	cv::Point2f intersection;
//	bool do_lines_intersect = apm::internal::LineIntersection(p1, p2, p3, p4, intersection);
//
//	ASSERT_TRUE(do_lines_intersect);
//	ASSERT_EQ(cv::Point2f(5, 5), intersection);
//}
///*
//TEST(FindCornersTest, PerfectSquare) {
//	std::vector<cv::Vec4i> lines;
//	lines.push_back(cv::Vec4i(5, 5, 5, 10));
//	lines.push_back(cv::Vec4i(5, 10, 10, 10));
//	lines.push_back(cv::Vec4i(10, 10, 10, 5));
//	lines.push_back(cv::Vec4i(10, 5, 5, 5));
//	std::vector<cv::Point2i> corners;
//
//	double avg_dimension = 300;
//	apm::internal::FindIntersections(lines, corners, 20, avg_dimension * 0.15);
//
//	ASSERT_EQ(4, corners.size());
//	ASSERT_NE(corners.end(), std::find(corners.begin(), corners.end(), cv::Point2i(5, 5)));
//	ASSERT_NE(corners.end(), std::find(corners.begin(), corners.end(), cv::Point2i(5, 10)));
//	ASSERT_NE(corners.end(), std::find(corners.begin(), corners.end(), cv::Point2i(10, 10)));
//	ASSERT_NE(corners.end(), std::find(corners.begin(), corners.end(), cv::Point2i(10, 5)));
//}
//
//TEST(FindCornersTest, SquareWithOutliers) {
//	std::vector<cv::Vec4i> lines;
//	lines.push_back(cv::Vec4i(0, 0, 90, 0));
//	lines.push_back(cv::Vec4i(100, 0, 100, 103));
//	lines.push_back(cv::Vec4i(100, 100, 5, 100));
//	lines.push_back(cv::Vec4i(0, 100, 0, 10));
//	lines.push_back(cv::Vec4i(120, 50, 200, 50));
//	std::vector<cv::Point2i> corners;
//
//	double avg_dimension = 300;
//	apm::internal::FindIntersections(lines, corners, 20, avg_dimension * 0.15);
//
//	ASSERT_EQ(4, corners.size());
//	ASSERT_NE(corners.end(), std::find(corners.begin(), corners.end(), cv::Point2i(0, 0)));
//	ASSERT_NE(corners.end(), std::find(corners.begin(), corners.end(), cv::Point2i(100, 0)));
//	ASSERT_NE(corners.end(), std::find(corners.begin(), corners.end(), cv::Point2i(100, 100)));
//	ASSERT_NE(corners.end(), std::find(corners.begin(), corners.end(), cv::Point2i(0, 100)));
//}
//
//TEST(FindCornersTest, NonPerpendicular) {
//	std::vector<cv::Vec4i> lines;
//	lines.push_back(cv::Vec4i(0, 0, 90, 0));
//	lines.push_back(cv::Vec4i(100, 0, 90, 90));
//	lines.push_back(cv::Vec4i(100, 100, 5, 100));
//	lines.push_back(cv::Vec4i(10, 90, 0, 10));
//	std::vector<cv::Point2i> corners;
//
//	double avg_dimension = 300;
//	apm::internal::FindIntersections(lines, corners, 20, avg_dimension * 0.15);
//
//	ASSERT_EQ(4, corners.size());
//}*/
//
