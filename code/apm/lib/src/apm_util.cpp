#include "../include/apm_util.h"

namespace automatic_package_measuring {

namespace internal {

void NearestNeighbours(std::vector<cv::Point2f>& points, int num_neighbours, std::vector<std::vector<int>>& neighbour_list_out) {

	cv::Mat_<float> flannPoints(points.size(), 2);
	for (int i = 0; i < points.size(); ++i) {
		flannPoints[i][0] = (double) points[i].x;
		flannPoints[i][1] = (double) points[i].y;
	}

	cv::flann::Index flann_index(flannPoints, cv::flann::LinearIndexParams());
	std::vector<int> indices = std::vector<int>(num_neighbours);
	std::vector<double> distances = std::vector<double>(num_neighbours);
	std::vector<double> query = std::vector<double>(2);

	for (int i = 0; i < points.size(); ++i) {

		query[0] = (double) points[i].x;
		query[1] = (double) points[i].y;

		flann_index.knnSearch(query, indices, distances, 1, cv::flann::SearchParams());

		neighbour_list_out.push_back(indices);
	}
}

} /* namespace internal */
} /* namespace automatic_package_measuring */

