#include "../include/paper_detection.h"
#include "../include/paper_detection_internal.h"
#include "../include/image_processing.h"
#include "../include/package_detection_internal.h"
#include <limits>
#include <cmath>
#include <set>
#include <algorithm>
#include <queue>
#include <chrono>
using namespace automatic_package_measuring::internal;
namespace automatic_package_measuring {
std::vector<cv::Point2f> FindPaper(const cv::Mat& image, const cv::Mat& edges) {

	cv::Mat edges_cpy = edges.clone();
	std::vector<std::vector<cv::Point>> contours;
	FindContours(edges_cpy, false, contours);
	double min_image_dimension = std::min(image.size().width, image.size().height);
	PruneShortContours(contours, min_image_dimension * MIN_PAPER_CONTOUR_LENGTH);
	PrunePeripheralContours(contours, image.size());

	if (contours.empty())
		return std::vector<cv::Point2f>();
	cv::Mat contours_mat = cv::Mat(image.size(), CV_8UC1, cv::Scalar(0));
	cv::drawContours(contours_mat, contours, -1, cv::Scalar(255));

	std::vector<cv::Vec4i> lines;
	DetectLines(contours_mat, lines);

	std::vector<std::vector<cv::Point2f>> papers = FindPapers(lines, image.size());

	double max_score = -std::numeric_limits<double>::max();
	std::vector<cv::Point2f> max_paper;
	cv::Mat eq_image;
	image.copyTo(eq_image);
	cv::equalizeHist(image, eq_image);
	for (auto it = papers.begin(); it != papers.end(); ++it) {
		double score = RatePaper(eq_image, lines, *it);
		if (score > max_score) {
			max_score = score;
			max_paper = *it;
		}

	}
	if (max_score < MIN_PAPER_ACCEPTED_SCORE)
		return std::vector<cv::Point2f>();

	return max_paper;

}

namespace internal {

double RatePaper(const cv::Mat& image, const std::vector<cv::Vec4i>& lines,
		const std::vector<cv::Point2f>& paper) {

	double angle_score = 100.0;
	double length_score = 100.0;
	int num_corners = paper.size();
	cv::Point prev = paper[0];
	for (int i = 1; i <= num_corners / 2; ++i) {
		cv::Point cur = paper[i];
		cv::Point opp1 = paper[(i + 1) % num_corners];
		cv::Point opp2 = paper[(i + 2) % num_corners];

		double length1 = cv::norm(cur - prev);
		double length2 = cv::norm(opp2 - opp1);

		double avg_length = (length1 + length2) / 2;

		double len_diff = std::min(length1, length2) / std::max(length1, length2);

		length_score *= len_diff;

		double angle = LineSegmentAngle(cv::Vec4i(prev.x, prev.y, cur.x, cur.y),
				cv::Vec4i(opp1.x, opp1.y, opp2.x, opp2.y));

		angle_score *= (1.0 - angle / 90.0);

		prev = cur;
	}

	double side1 = (cv::norm(paper[0] - paper[1]) + cv::norm(paper[2] - paper[3])) / 2;
	double side2 = (cv::norm(paper[1] - paper[2]) + cv::norm(paper[3] - paper[0])) / 2;
	double long_side = std::max(side1, side2);
	double short_side = std::min(side1, side2);

	double ratio = long_side / short_side;

	double ratio_score = std::min(ratio, A4_LENGTH_RATIO) / std::max(ratio, A4_LENGTH_RATIO) * 100;

	cv::Rect bounding_rect = cv::boundingRect(paper);
	std::vector<cv::Point> roi_paper;
	for (auto point : paper) {
		point.x -= bounding_rect.x;
		point.y -= bounding_rect.y;
		roi_paper.push_back(point);
	}

	cv::Mat roi = cv::Mat(bounding_rect.size(), image.type(), cv::Scalar(0));

	cv::fillConvexPoly(roi, roi_paper, cv::Scalar(255));

	cv::Mat tmp_roi = cv::Mat(image, bounding_rect);

	tmp_roi.copyTo(roi, roi);

	cv::Mat histogram;
	int channels[] = { 0 };
	int bins[] = { 30 };
	const float *ranges[1];
	float range[] = { 0.0f, 255.0f };
	ranges[0] = range;
	cv::calcHist(&roi, 1, channels, cv::Mat(), histogram, 1, bins, ranges);

	double top_bin = histogram.at<float>(bins[0] - 1) / ((double) (cv::contourArea(roi_paper)));
	double color_score = 400.0 * top_bin;

	if (angle_score < MIN_PAPER_ACCEPTED_SUBSCORE || length_score < MIN_PAPER_ACCEPTED_SUBSCORE
			|| color_score < MIN_PAPER_ACCEPTED_SUBSCORE || ratio_score < MIN_PAPER_ACCEPTED_SUBSCORE)
		return 0;

	return (angle_score + length_score + color_score + ratio_score) / 4;
}

int FindTopCorner(const std::vector<cv::Point2f>& corners) { // assumes corners is in clockwise order
	int top_corner = -1;
	int top_corner_y = std::numeric_limits<int>::max(); // y axis points down so top corner has smallest y.

	for (int i = 0; i < corners.size(); ++i) {
		if (corners[i].y < top_corner_y) {
			top_corner = i;
			top_corner_y = corners[i].y;
		}
	}

	return top_corner;
}

std::vector<std::vector<cv::Point2f>> FindPapers(const std::vector<cv::Vec4i>& lines,
		const cv::Size& image_size) {

	std::vector<std::tuple<int, int>> line_pairs;
	int min_image_dimension = std::min(image_size.width, image_size.height);
	FindParallelLines(lines, MIN_PAPER_PARALLEL_LINE_DIST * min_image_dimension, line_pairs);
	std::vector<std::vector<cv::Point2f>> papers;
	if (line_pairs.size() < 2)
		return papers;

	if (line_pairs.size() > MAX_PAPER_LINE_PAIRS) {
		std::cout << "Paper detection: Too many line pairs: " << line_pairs.size() << std::endl;
		return std::vector<std::vector<cv::Point2f>>();
	}

	for (int i = 0; i < line_pairs.size(); ++i) {
		for (int j = i + 1; j < line_pairs.size(); ++j) {
			std::vector<int> line_indices = GetLinesInPairs(line_pairs, { i, j });
			if (line_indices.empty())
				continue;
			std::vector<cv::Point2f> paper;
			bool is_paper_valid = TryToCreatePolygon(lines, line_indices, image_size, paper);

			if (!is_paper_valid)
				continue;
			papers.push_back(paper);
		}
	}
	return papers;
}

// TODO make const when tuning is finished
double A4_LONG_SIDE = 297;
double A4_SHORT_SIDE = 210;
double A4_LENGTH_RATIO = A4_LONG_SIDE / A4_SHORT_SIDE;
double MIN_PAPER_PARALLEL_LINE_DIST = 0.02;
double MIN_PAPER_CONTOUR_LENGTH = 0.005;
int MAX_PAPER_LINE_PAIRS = 200;
double MIN_PAPER_ACCEPTED_SCORE = 80;
double MIN_PAPER_ACCEPTED_SUBSCORE = 60;

} /* namespace internal */

} /* namespace automatic_package_measuring */
